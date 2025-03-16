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
const char* mqtt_topic = "machines/1001-1001-1001-1001/aa/value";
const char* mqtt_init_topic = "valueGet/channelCount/0001"; // 暂定此主机编号为0001

static bool sensorNumber = false;
int SENSOR_NUMBER = 0;

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
bool openConfirm = 0;
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
        //snprintf(mqttMessage, sizeof(mqttMessage), "Transmitter %d: Voltage = %.3f V", transmitterNumber, voltage);
        if (i == 0) mqtt_topic = "machines/1001-1001-1001-1001/a0/value";
        if (i == 1) mqtt_topic = "machines/1001-1001-1001-1001/a1/value";
        if (i == 2) mqtt_topic = "machines/1001-1001-1001-1001/a2/value";
        if (i == 3) mqtt_topic = "machines/1001-1001-1001-1001/a3/value";
        snprintf(mqttMessage, sizeof(mqttMessage), "[%.3f][1]", voltage);
        client.publish(mqtt_topic, mqttMessage);
    }
}
// 回调函数
void messageReceived(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // 处理消息内容
    char message[length + 1];
    for (unsigned int i = 0; i < length; i++) {
        message[i] = (char)payload[i];
    }
    message[length] = '\0';  // 添加字符串结束符

    if (strcmp(message, "1") == 0) {
        SENSOR_NUMBER = 0x01;
        sensorNumber = true;
    } else if (strcmp(message, "2") == 0) {
        SENSOR_NUMBER = 0x02;
        sensorNumber = true;
    } else if (strcmp(message, "3") == 0) {
        SENSOR_NUMBER = 0x03;
        sensorNumber = true;
    } else if (strcmp(message, "4") == 0) {
        SENSOR_NUMBER = 0x04;
        sensorNumber = true;
    } else {
        Serial.println("Unknown command");
        client.publish(mqtt_init_topic, "Unknown, Init again");
    }
}
void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    reconnect();
    
}

void loop() {
    if (client.connected() && openConfirm == 0){
      openConfirm = 1;
      client.publish(mqtt_init_topic, "Init");
      client.subscribe("response_topic"); 
    }

    client.loop();
    // 检查 MQTT 连接
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    client.setCallback(messageReceived);  // 设置回调函数


    if ( sensorNumber == true )
    {
        // 构造Modbus帧
        uint8_t frame[12];  // 帧最大长度
        int frameLength = 0;
    
        // 字头
        frame[frameLength++] = FRAME_HEADER;
    
        // 设备地址
        frame[frameLength++] = DEVICE_ADDRESS;
    
        // 功能码
        frame[frameLength++] = FUNCTION_CODE;
    
        // 数据长度
        frame[frameLength++] = DATA_LENGTH;
    
        // CRC校验码
        uint16_t crc = calculateCRC16(frame, frameLength);
        frame[frameLength++] = crc & 0xFF;           // CRC低字节
        frame[frameLength++] = (crc >> 8) & 0xFF;    // CRC高字节
        
        // 传感器数量
        frame[frameLength++] = SENSOR_NUMBER;
        
        // 添加字尾
        frame[frameLength++] = FRAME_TAIL;
    
        Serial.write(frame, frameLength);

        delay(10000);  
    }
}
