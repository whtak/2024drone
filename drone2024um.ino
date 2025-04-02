#include <DHT.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <RtcDS1302.h>


// 먼지센서변수모음 줄여서 먼센변모
int measurePin = 0;          // 먼지 센서의 측정 핀
int ledPower = 2;            // 먼지 센서의 LED 전원 핀
int samplingTime = 280;      // 샘플링 시간 (마이크로초)
int deltaTime = 40;          // 델타 시간 (마이크로초)
int sleepTime = 9680;        // 대기 시간 (마이크로초)
float voMeasured = 0;        // 측정된 전압 값
float calcVoltage = 0;       // 계산된 전압 값
float dustDensity = 0;       // 먼지 농도 값

// 각 센서 당 1분씩
const int measurementDuration = 60000; // 측정 기간: 1분 (60초 * 1000밀리초)
const int measurementInterval = 1000;  // 측정 간격: 1초

// 통신 뭐시기 모음 줄여서 통뭐음
#define CE_PIN  9
#define CSN_PIN 10
const byte addressB[6] = "00001"; // 아두이노 B의 주소
RF24 radio(CE_PIN, CSN_PIN);
char FinalDataToSend[50] = "";

// 클럭모듈 변수 모음 줄여서 클변모
ThreeWire myWire(6,5,7);    // DS1302  객체 설정
RtcDS1302<ThreeWire> Rtc(myWire);
RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

// 온습도 뭐시기 모음 줄여서  온기모
#define DHTPIN 8     // 온습도 디지털8
#define DHTTYPE DHT22   // DHT 타입 22로 설정
DHT Dht(DHTPIN, DHTTYPE);
int maxHum = 60;
int maxTemp = 40;    // 온습도 변수


void setup() {
  Serial.begin(9600);        // 시리얼 통신 초기화 (9600 bps)
  pinMode(ledPower, OUTPUT); // LED 전원 핀을 출력으로 설정

  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);
  Rtc.Begin();
  printDateTime(compiled);
  Serial.println();

    if (!Rtc.IsDateTimeValid()) 
    {
        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }
    if (Rtc.GetIsWriteProtected())
    {
        Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }
    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }
    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
  
  Dht.begin();    //온습도 시작
  delay(2000);

  float soopdo = Dht.readHumidity();
  float ondo = Dht.readTemperature();

  if (!radio.begin()) {
      Serial.println("RF24_Jotdam");   // 통신 모듈 연결 확인
      while (1) {}
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(76); // 채널 설정 (0-125)
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(addressB);
  Serial.println("RF24_JunbiWanlyo");
  
}

void loop() {
  RtcDateTime t = Rtc.GetDateTime();
  printDateTime(t);
  Serial.println();
  // 현재 시간 가져오기
  
  // 지정한 시간 (예: 12시 38분)
  int targetHour = 16;
  int targetMinute = 40;
  
  // 현재 시간이 목표 시간과 일치하는지 확인
  if (t.Hour() == targetHour && t.Minute() == targetMinute) {
    // 기존 loop 내용을 여기에 넣습니다
    AllSensorStart();
  } else {
    // 목표 시간이 아닐 때는 대기
    Serial.println("Waiting for target time...");
    delay(30000); // 30초마다 확인
  }
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[26];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

void AllSensorStart() {
  
  delay(1000);
  Serial.println("Dust_Sensor_Start_In");
  printDateTime(compiled);

  float totalDustDensity = 0;  // 총 먼지 농도 합계
  int dustValidMeasurements = 0;   // 유효한 측정 횟수
  unsigned long startTime = millis();  // 측정 시작 시간


  // 미세먼지 1분 시작
  for (int i = 1; i <= 60; i++) {  // 60초 동안 반복
    digitalWrite(ledPower, LOW);  // LED 켜기
    delayMicroseconds(samplingTime);  // 샘플링 시간 대기

    voMeasured = analogRead(measurePin);  // 아날로그 값 읽기

    delayMicroseconds(deltaTime);  // 델타 시간 대기
    digitalWrite(ledPower, HIGH);  // LED 끄기
    delayMicroseconds(sleepTime);  // 대기 시간

    // 전압 계산
    calcVoltage = voMeasured * (5.0 / 1024.0);
    // 먼지 농도 계산 (ug/m3)
    dustDensity = (0.17 * calcVoltage - 0.1) * 1000;

    // 현재 먼지 농도 출력
    Serial.print(i);
    Serial.print("초 - 먼지 농도: ");
    Serial.print(dustDensity);
    Serial.println(" ug/m3");

    totalDustDensity += dustDensity;
    dustValidMeasurements++;

    delay(measurementInterval - (samplingTime + deltaTime + sleepTime) / 1000);  // 정확히 1초가 되도록 조정
  }

  // 평균 먼지 농도 계산 및 출력
  float averageDustDensity = totalDustDensity / dustValidMeasurements;
  Serial.println("\n===== 측정 결과 =====");
  Serial.print("1분 평균 먼지 농도: ");
  Serial.print(averageDustDensity, 2);  // 소수점 둘째 자리까지 출력
  Serial.println(" ug/m3");
  Serial.print("유효 측정 횟수: ");
  Serial.println(dustValidMeasurements);

  delay(1000);
  
  float totalHumidity = 0;    // 온습도 평균 계산할 때 쓰는 변수
  float totalTemperature = 0;
  int dhtValidMeasurements = 0;

// 온습도 1분 시작
  Serial.println("DHT22_Sensor_Start_In");
  printDateTime(compiled);
  
  startTime = millis();  // startTime 재설정

  for (int i = 1; i <= 60; i++) {  // 60초 동안 반복
    float humidity = Dht.readHumidity();
    float temperature = Dht.readTemperature();

    if (!isnan(humidity) && !isnan(temperature)) {
      totalHumidity += humidity;
      totalTemperature += temperature;
      dhtValidMeasurements++;

      Serial.print(i);
      Serial.print("초 - 습도: ");
      Serial.print(humidity);
      Serial.print("%, 온도: ");
      Serial.print(temperature);
      Serial.println("°C");
    } else {
      Serial.println("DHT22 센서 읽기 실패");
    }

    delay(measurementInterval);  // 1초 대기
  }

  // 평균 온습도 계산 및 출력
  float averageHumidity = totalHumidity / dhtValidMeasurements;
  float averageTemperature = totalTemperature / dhtValidMeasurements;
  Serial.println("\n===== 1분 측정 결과 =====");
  Serial.print("평균 습도: ");
  Serial.print(averageHumidity);
  Serial.println("%");
  Serial.print("평균 온도: ");
  Serial.print(averageTemperature);
  Serial.println("°C");
  Serial.print("유효 측정 횟수: ");
  Serial.println(dhtValidMeasurements);

  delay(1000);

  long UVtotal = 0;
  Serial.println("UV_Sensor_Start_In");
  printDateTime(compiled);

  for (int i = 1; i <= 60; i++) {  // 정확히 60번 반복
    int UVvalue = analogRead(1);
    UVtotal += UVvalue;

    Serial.print(i);
    Serial.print("초 - 자외선 지수: ");
    Serial.println(UVvalue);

    delay(measurementInterval);
  }

  float UVaverage = (float)UVtotal / 60;
  Serial.println("\n===== 측정 결과 =====");
  Serial.print("1분 평균 값: ");
  Serial.println(UVaverage);
  Serial.println("총 측정 횟수: 60");

  delay(1000);

    char AllSensorData[32]; // 충분한 크기의 char 배열 선언

    // dtostrf를 사용하여 각 float 값을 char 배열에 추가
    char dustDensityStr[8];
    char humidityStr[8];
    char temperatureStr[8];
    char uvStr[8];

    dtostrf(averageDustDensity, 4, 1, dustDensityStr); // 소수점 첫째 자리
    dtostrf(averageHumidity, 4, 1, humidityStr);
    dtostrf(averageTemperature, 4, 1, temperatureStr);
    dtostrf(UVaverage, 4, 1, uvStr);

    // 최종 문자열 생성
    snprintf(AllSensorData, sizeof(AllSensorData), "%s,%s,%s,%s", 
             dustDensityStr, humidityStr, temperatureStr, uvStr);

    Serial.println(AllSensorData); // 결과 출력
  
  bool result = false;

  while (!result) {
    radio.stopListening(); // 송신 모드로 전환
    Serial.println("Sending data...");
    result = radio.write(AllSensorData, strlen(AllSensorData) + 1); // 수정된 부분
    
    if (result) {
        Serial.println("Data sent successfully: ");
        Serial.println(AllSensorData);
    } else {
        Serial.println("Failed to send data");
    }
    delay(10000);
  }
  delay(1000);
}