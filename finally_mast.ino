#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

// WiFi 配置
const char* ssid = "x";  // WiFi
const char* password = "x";  // 密码

// MQTT 配置
const char* mqtt_server = "112.4.115.127";  // MQTT 服务器地址
const int mqtt_port = 1883;  // MQTT 服务器端口
const char* mqtt_client_id = "ESP8266Client";  // MQTT 客户端 ID
const char* mqtt_topic = "machines/1001/2333/value";  // MQTT 发布的主题

WiFiClient espClient;
PubSubClient client(espClient);

// 自定义CRC16校验函数
uint16_t calculateCRC16(uint8_t* data, int length) {
    uint16_t crc = 0xFFFF;  // 初始值
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;  // CRC-16-Modbus多项式
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// MQTT 重连函数
void reconnect() {
    while (!client.connected()) {
        Serial.print("尝试连接 MQTT...");
        if (client.connect(mqtt_client_id)) {
            Serial.println("连接成功！");
        } else {
            Serial.print("连接失败，错误码：");
            Serial.print(client.state());
            delay(5000);
        }
    }
}

// 定义Modbus帧的结构
const int FRAME_HEADER = 0xAD;  // 字头
const int FRAME_TAIL = 0xAF;    // 字尾
const int DEVICE_ADDRESS = 0x01;  // 设备地址
const int FUNCTION_CODE = 0x03;  // 功能码（轮询功能码示例）
const int DATA_LENGTH = 0x00;    // 数据长度，轮询时为0

// WiFi 连接函数
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("连接到 WiFi：");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        Serial.print(WiFi.status());  // 打印WiFi状态码
    }
    Serial.println("");
    Serial.println("WiFi 连接成功！");
    Serial.println(WiFi.localIP());
}

// 验证接收到的Modbus帧
bool validateFrame(uint8_t* frame, int length) {
    // 确保帧长度至少包含头、地址、功能码、字节数、CRC和尾
    if (length < 9) return false;

    // 提取帧的有效部分（不包括帧尾）
    int dataLength = length - 1;  // 排除帧尾
    uint16_t receivedCRC = (frame[dataLength - 1] << 8) | frame[dataLength - 2];
    uint16_t calculatedCRC = calculateCRC16(frame, dataLength - 2);  // 不包括CRC本身

    // 比较接收到的CRC和计算的CRC
    return (receivedCRC == calculatedCRC);
}

// 解析Modbus帧并提取传感器数据
void parseModbusFrame(uint8_t* frame, int length) {
    // 提取字节数
    int dataLength = frame[3];  // 字节数字段
    int transmitterCount = dataLength / 3;  // 每个变送器占用3字节

    Serial.print("Number of Transmitters: ");
    Serial.println(transmitterCount);



    // 提取每个变送器的数据
    for (int i = 0; i < transmitterCount; i++) {
        int index = 4 + i * 3;  // 数据起始位置
        int transmitterNumber = frame[index];  // 变送器序号
        uint16_t adcValue = (frame[index + 1] << 8) | frame[index + 2];  // 16位ADC值

        // 假设参考电压为3.3V，ADC分辨率为65535
        const float referenceVoltage = 3.3;
        const int adcResolution = 65535;
        float voltage = (adcValue / (float)adcResolution) * referenceVoltage;

        Serial.print("Transmitter ");
        Serial.print(transmitterNumber);
        Serial.print(": ADC Value = ");
        Serial.print(adcValue);
        Serial.print(", Voltage = ");
        Serial.print(voltage, 3);  // 保留3位小数
        Serial.println(" V");

        // 将解析结果发送到MQTT
        char mqttMessage[100];
        snprintf(mqttMessage, sizeof(mqttMessage), "Transmitter %d: Voltage = %.3f V", transmitterNumber, voltage);
        client.publish(mqtt_topic, mqttMessage);
    }
}

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    reconnect();
}

void loop() {
    // 检查 MQTT 连接
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // 构造Modbus帧
    uint8_t frame[10];  // 帧最大长度
    int frameLength = 0;

    // 添加字头
    frame[frameLength++] = FRAME_HEADER;

    // 添加设备地址
    frame[frameLength++] = DEVICE_ADDRESS;

    // 添加功能码
    frame[frameLength++] = FUNCTION_CODE;

    // 添加数据长度
    frame[frameLength++] = DATA_LENGTH;

    // 计算CRC校验码
    uint16_t crc = calculateCRC16(frame, frameLength);
    frame[frameLength++] = crc & 0xFF;           // CRC低字节
    frame[frameLength++] = (crc >> 8) & 0xFF;    // CRC高字节

    // 添加字尾
    frame[frameLength++] = FRAME_TAIL;

    // 发送Modbus帧
    Serial.write(frame, frameLength);



    // 等待一段时间
    delay(10000);  // 
}
