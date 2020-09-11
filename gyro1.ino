
/*
   손목 gyro2로부터 movcount 받음, 안드로이드와 통신 부분
*/

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Wire.h>
#include "Kalman.h"
#include <SoftwareSerial.h>


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


int TX = 2; //TX핀 D2
int RX = 3; //RX핀 D3
SoftwareSerial mySerial(2, 3);

unsigned int recmov = 0;  //팔꿈치 데이터 수신 판별
bool movcount = false;  //팔꿈치가 움직였는가
int i = 0; //while문 반복 조건
int count;  //총 횟수
int score;  //유효점수
int rec;  //안드로이드로 부터 받은 값 (시작 종료 판별)
bool jud1, jud2, jud3;  //팔을 올릴때와 내릴때 해당 상태 판별, jud3 = 중복 카운트 방지
double N1, N2, P1, P2; //N1,P1 팔을 올릴때 현재,과거값 N2,P2 팔을 내릴때 현재,과거값 저장
double gyro;
double standpick; //최고점 기준
double pick1, pick2;  //pick1 - 최저점 기록, pick2 - 최고점 기록



void setup()
{
  Serial.begin(9600);

  Mirf.cePin = 9;   //CE핀
  Mirf.csnPin = 10; //CS핀
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();  // nRF24L01 초기화

  //수신측 어드레스 설정
  Mirf.setRADDR((byte *)"TX_01");
  //송수신에 사용할 바이트 수 지정 sizeof(unsigned int) 는 2바이트
  Mirf.payload = sizeof(unsigned int);
  //송수신 채널 지정(0~128), 송수신 모듈이 동일한 채널을 사용해야함
  Mirf.channel = 3;
  Mirf.config();

  //수신기표시
  Serial.println("NRF24L01_RECEIVE");

  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);

  }

  delay(100); // Wait for sensor to stabilize


  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
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

void loop()
{
  //수신할 데이터 버퍼 정의
  byte data[Mirf.payload];

  rec = mySerial.read();  //블루투스를 통한 데이터 수신

  if (rec == 1) { //안드로이드로부터 1값을 전송받으면 시작

    Serial.println("시작");

    count  = 0; //횟수 초기화
    score = 0;  //점수 초기화
    jud1 = true; //올라가는 최초의 순간을 판단하기 위해 true
    jud2 = false; //시작부터 내렸을 때 내린 지점을 저장하지 않기 위해 false
    jud3 = false; //카운트 중복 방지(팔을 내린 후 부터 ture)
    i = 1;  //시작버튼(rec == 1)을 눌렀을 때 측정 시작

    //반복(시작)
    while ( i == 1 ) { //i의 값이 1일 때 반복

      rec = mySerial.read();  //안드로이드로부터 종료 신호 수신대기

      angleValue(); //손목 자이로값 측정

      gyro = kalAngleY;  //실시간 손목의 값을 gyro에 저장

      //팔을 올릴 때(시작)
      N1 = gyro; //현재변수 N1에 현재값 저장
      N2 = gyro;  //현재변수 N2에 현재값 저장

      if (P1 - N1 < 0 && jud1 == true) { //팔을 올리는 순간 (과거 - 현재 < 0), 최초로 팔을 올리는 것진지 판단

        if (N1 < 270 || N1 > 0) { //일정 범위 내에서만 판단

          pick1 = N1;  //pick1에 현재값 저장(최저점)

          jud1 = false; //팔을 올리는 최초의 순간만 기록하기 위함
          jud2 = true;  //팔을 올린 후 최초로 팔을 내릴 때를 판단하기 위함
          jud = false;
        }
      } //팔을 올릴 때(끝)


      //팔을 내릴 때(시작)
      if ( N2 - P2 < 0 && jud2 == true) { //팔을 내리는 순간 (현재 - 과거 < 0), 최초로 팔을 내리는 것진지 판단

        if (N2 < 270 || N2 > 0) { //일정 범위 밖에서의 움직임은 측정하지 않음
          pick2 = N2; //pick2에 현재값 저장(최고점)

          jud1 = true;  //팔을 내린 후 최초로 팔을 올릴 때를 판단하기 위함
          jud2 = false; //팔을 내리는 최초의 순간만 기록하기 위함
          jud3 = true;  //손목의 최초 위치에 도달했을 경우 카운트를 세기 위함
        }
      } //팔을 내릴 때(끝)

      P1 = N1;  //현재값 N1을 과거값 P1에 저장 - 팔을 올릴 때
      P2 = N2;  //현재값 N2를 과거값 P2에 저장 - 팔을 내릴 때

      //팔꿈치 데이터 수신
      if (Mirf.dataReady()) {  //데이터수신 대기
        Mirf.getData(data);    //수신 데이터를 data에 저장
        //데이터 버퍼를 통합
        recmov = (unsigned int)((data[1] << 8) | data[0]);
        if (recmov == 1) {
          movcount = true;
        }
      }

      //카운트(시작)
      if (pick1 <= gyro && jud3 == true) { //현재 손목의 위치가 최초의 위치(pick1)에 도달하는 순간 카운트

        count += 1; //count +1

        if (pick2 < standpick) { //사용자의 최고점이 기준데이터의 최고점보다 낮으면(최고점에 도달하지 않고 팔을 내리면)
          score += 0; //점수를 주지 않음
        }
        else if ( pick2 > standpick) { //최고점이 기준보다 크고
          if (movcount == true) { //팔꿈치가 움직인 경우
            score += 0; //점수를 주지 않음
          }
          else {  //팔꿈치가 움직이지 않고 최고점의 위치에 도달하고 팔을 내린 경우(정상운동)
            score += 1; //점수 1점 부여 -> 최고점의 위치 정확, 팔꿈치가 움직이지 않음
          }
        }
        jud3 = false; //최초1회만 카운트 (중복 카운트 방지)
        movcount = false; //1회 카운트 후 팔꿈치 움직임 여부 초기화
      } //카운트(끝)

      if ( rec == 2 ) { //안드로이드로부터 종료(rec == 2)를 받았을 경우
        Serial.println("종료");
        i = 2;  //반복문 종료(측정 종료)
      }

    } //while(i==1)

  } //if(rec == 1)
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
