//arduino_final

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

const int rxPin = 2;  // RX 引脚连接到数字引脚 2
const int txPin = 3;  // TX 引脚连接到数字引脚 3

SoftwareSerial mySerial(rxPin, txPin);

Adafruit_ADS1115 ads;
#define FILTER_SIZE 20
int16_t adcBuffer[FILTER_SIZE];
uint8_t bufIndex = 0;


// CRC16校验函数
uint16_t calculateCRC16(uint8_t* data, int length) {
    uint16_t crc = 0xFFFF; 
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;  
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ADC滤波
int16_t readFilteredADC(int a) {
  adcBuffer[bufIndex] = ads.readADC_SingleEnded(a);
  bufIndex = (bufIndex + 1) % FILTER_SIZE;
  int32_t sum = 0;
  for (uint8_t i = 0; i < FILTER_SIZE; i++) sum += adcBuffer[i];
  return sum / FILTER_SIZE;
}


// Modbus帧
const int FRAME_HEADER = 0xAD; 
const int FRAME_TAIL = 0xAF;    
const int DEVICE_ADDRESS = 0x01;  
const int FUNCTION_CODE = 0x03;  
const int DATA_LENGTH = 0x00;    

// ADC相关参数
const float referenceVoltage = 3.3;  // ADC参考电压
const int adcResolution = 65535;  // 16位ADC分辨率

int TRANSMITTER_COUNT = 0;

int getSensorNumber(uint8_t* frame, int length) {
  unsigned char transmitterCountByte = frame[length - 1]; 
  return transmitterCountByte;
}

        
void setup() {
    Serial.begin(115200);
    mySerial.begin(115200);
    ads.begin(0x48);   
    ads.setGain(GAIN_ONE); // ±4.096V
}

void loop() {

    if (Serial.available() > 0) {
        uint8_t receivedByte = Serial.read();

        static bool frameStart = false;
        static uint8_t frameBuffer[256]; 
        static int frameIndex = 0;

        if (receivedByte == FRAME_HEADER) {
            frameStart = true;
            frameIndex = 0;
            frameBuffer[frameIndex++] = receivedByte;
        } else if (frameStart) {
            frameBuffer[frameIndex++] = receivedByte;

            if (receivedByte == FRAME_TAIL) {  
                delay(500);   //可调大，目的是防止半双工485通信冲突
                frameStart = false;
                TRANSMITTER_COUNT = getSensorNumber(frameBuffer, frameIndex);
                sendModbusResponse();
            }
        }
    }

    delay(100); 
}

// 验证接收到的Modbus帧
bool validateFrame(uint8_t* frame, int length) {
    // 确保帧长度至少包含头、地址、功能码、数据长度、传感器数量、CRC和尾
    if (length < 9) return false;

    int dataLength = length - 1;  // 排除帧尾
    uint16_t receivedCRC = (frame[dataLength - 1] << 8) | frame[dataLength - 2];
    uint16_t calculatedCRC = calculateCRC16(frame, dataLength - 2);  

    return (receivedCRC == calculatedCRC);
}
void sendModbusResponse() {
    uint8_t responseFrame[256];  // 响应帧缓冲区
    int responseLength = 0;
    
    if (TRANSMITTER_COUNT == 0 || TRANSMITTER_COUNT > 4 ) {
    Serial.println("Error: No transmitters to process.");
    return;
}   
    // 构造响应帧
    responseFrame[responseLength++] = FRAME_HEADER;  
    responseFrame[responseLength++] = DEVICE_ADDRESS; 
    responseFrame[responseLength++] = FUNCTION_CODE;  

    // 添加字节数（变送器数量 * 3）
    responseLength += 1;
    responseFrame[3] = TRANSMITTER_COUNT * 3;

    // 模拟电压值并转换为16位ADC值
    for (int i = 0; i < TRANSMITTER_COUNT; i++) {
        int16_t adcValue = readFilteredADC(i); // 调用函数读取
        
        Serial.print("Transmitter ");
        Serial.print(i + 1);

        
        Serial.print(" V -> ADC Value = ");
        Serial.println(adcValue);

        responseFrame[responseLength++] = i + 1;  // 变送器序号
        responseFrame[responseLength++] = adcValue >> 8;  // ADC值高字节
        responseFrame[responseLength++] = adcValue & 0xFF;  // ADC值低字节
    }

    

    // 计算CRC校验码
    uint16_t crc = calculateCRC16(responseFrame, responseLength);
    responseFrame[responseLength++] = crc & 0xFF;  // CRC低字节
    responseFrame[responseLength++] = (crc >> 8) & 0xFF;  // CRC高字节
    responseFrame[responseLength++] = FRAME_TAIL;  // 字尾

    // 发送响应帧
    mySerial.write(responseFrame, responseLength);
    //Serial.println("Response frame sent.");
}
