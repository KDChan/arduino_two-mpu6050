
/*
   팔꿈치 흔들림 정보 전송
*/

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Wire.h>
#include "Kalman.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; //가속도 x, y값
double temp; // Temperature
double gyroXangle, gyroYangle; //자이로센서의 x, y 축 값
double compAngleX, compAngleY; //상보필터
double kalAngleX, kalAngleY; // 칼만필터 적용 x, y

uint32_t timer;
uint8_t i2cData[14]; //i2c통신 버퍼 (14바이트)
////////////////////////////

int MinX; //팔꿈치 좌우 흔들림 기준값
int MaxY; //팔꿈치 상하 흔들림 기준값

void setup()
{
  Serial.begin(9600);

  Mirf.cePin = 9;   //CE핀
  Mirf.csnPin = 10; //CS핀
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();  // nRF24L01 초기화

  //송수신에 사용할 바이트 수 지정 sizeof(unsigned int) 는 2바이트
  Mirf.payload = sizeof(unsigned int);
  //송수신 채널 지정(0~128), 송수신 모듈이 동일한 채널을 사용해야함
  Mirf.channel = 3;
  Mirf.config();
  //송신기표시
  Serial.println("NRF24L01_SEND");

  Wire.begin();
  i2cData[0] = 7; // 샘플링속도 1000Hz - 8Hz(7+1) = 1000Hz 설정
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // 4개의 레지스터에 동시에 기록
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register

    while (1);

  }

  delay(100); // 센서 안정화를 위한 딜레이

  /* Set kalman and gyro starting angle */
  /* 칼만과 자이로의 시작 각도 설정 */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);

  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  //-파이 ~ +파이  =>  0 ~ 2파이값으로 변환
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();

}

unsigned int movcount = 0; //움직임 카운트 - 전송값

void loop()

{
  //송신할 데이터 버퍼 정의
  byte data[Mirf.payload];

  //아날로그 데이터를 송신할 바이트 어레이에 저장
  data[0] = movcount & 0xFF;   //하위바이트
  data[1] = movcount >> 8;     //상위바이트


  angleValue(); //팔꿈치의 자이로값 측정

  Serial.print(kalAngleX);
  Serial.print('\t');
  Serial.print(kalAngleY);
  Serial.println('\t');

  if (kalAngleX < 310 || kalAngleY > 310) { //기준보다 측정값이 낮을때(좌우 흔들림이 있을 때) 또는 기준보다 측정값이 높을때(상하 흔들림이 있을 때)
    movcount = 1; //movcount 1
    //알림(부저/진동)
  }

  if (movcount == 1) {
    //수신측 어드레스 지정 및 데이터 송신
    Mirf.setTADDR((byte *)"TX_01");
    Mirf.send(data);
    //데이터가 송신이 완료될 때가지 대기
    while (Mirf.isSending()) {}
    delay(20);
    movcount = 0; //전송 후 움직임 카운트 초기화
  }
  
  if(kalAngleX < 310 || kalAngleY < 310){ //팔꿈치가 흔들렸을 경우
    movcount = 0;
    Serial.println("팔꿈치 흔들림 감지");
    //알림 중지
  }

}

void angleValue() {
  /* Update all the values */
  /* 측정한 데이터를 받아옴 */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  //atan(높이,밑변) 으로 -파이~+파이의 라디안 값 반환 후 각도로 바꾸는 계산 (위 링크)
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0; //자이로 x축 각도 변화량
  double gyroYrate = -((double)gyroY / 131.0);  //y축
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // 필터보정이 없는 자이로 x각도
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000); // 필터보정이 없는 자이로 y각도

  //상보필터를 이용한 각도 계산
  //값의 계수 (0.93)는 실험으로 얻을 수 있는 값 -> 필요한 만큼 변화 가능
  compAngleX = (0.97 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.03 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.97 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.03 * accYangle);

  //칼만 필터를 이용한 각도 계산(칼만은 kalman.h 라이브러리를 통해 계산함)
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);



  timer = micros();

  temp = ((double)tempRaw + 12412.0) / 340.0;

}
