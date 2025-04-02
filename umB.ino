#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN 10

const byte addressA[6] = "00001"; // 아두이노 A의 주소
const byte addressC[6] = "00002"; // 아두이노 C의 주소

char receivedData[32]; // 수신할 데이터
RF24 radio(CE_PIN, CSN_PIN);
bool dataReceived = false;

void setup() {
    Serial.begin(9600);
    while (!Serial) {} // 시리얼 연결 대기
    
    Serial.println("Arduino B starting...");
    
    if (!radio.begin()) {
        Serial.println("Radio hardware not responding!");
        while (1) {} // 무한 루프
    }
    
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(76); // 채널 설정 (0-125)
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, addressA); // 아두이노 A로부터 수신
    radio.openWritingPipe(addressC); // 아두이노 C로 송신
    radio.startListening(); // 수신 모드 시작
    
    Serial.println("Arduino B ready to relay");
}

void loop() {
    if (!dataReceived && radio.available()) {
        radio.read(&receivedData, sizeof(receivedData)); // 데이터 수신
        Serial.print("Received from A: ");
        Serial.println(receivedData);
        dataReceived = true;
        radio.stopListening(); // 송신 모드로 전환
    }

    if (dataReceived) {
        bool sent = radio.write(&receivedData, sizeof(receivedData)); // 아두이노 C로 데이터 전송
        if (sent) {
            Serial.print("Forwarded to C: ");
            Serial.println(receivedData);
            dataReceived = false;
            radio.startListening(); // 다시 수신 모드로 전환
        } else {
            Serial.println("Failed to forward data to C, retrying...");
            // 재전송 시도를 위해 잠시 대기
            delay(100);
        }
    }
