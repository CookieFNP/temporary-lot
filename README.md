# temporary-lot

实现
- 主机：
  1.上电后访问服务器，连接成功则发送初始化topic，等待回复传感器数值
  2.将包含传感器数值（放置在了帧尾前一字节处）的modbus轮询发送至从机，目前间隔10s
  3.接收从机数据并解析传感器数值，topic发布
- 从机：
  1.接收主机modbus，开始执行传感器读取
  2.adc数据已进行软件滤波
  3.根据传感器数量生成不同长度modbus并发布


>若485模块因半双工通信且自动识别，导致收发数据有误 或 一端无法收发情况，请酌情更改或添加delay()延迟，
>确保收发不同时进行，避免因数据冲突造成的遗失。

>或者更改成全双工通信模块，将**有效**解决此问题。
