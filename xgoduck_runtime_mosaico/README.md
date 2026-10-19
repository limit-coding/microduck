# xgoduck runtime mosaico

xgoduck runtime mosaico 是 XGO Duck 在 [ESP-Mosaico V1.2](https://github.com/esp-mosaico/esp-mosaico-bsp)（ESP32-S31）上的运行代码。
行走策略由 [xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl) 训练，导出后转成 C 数组编进固件。
整体工程在 [Microduck](https://github.com/pollen-robotics/microduck)（Pollen Robotics）的基础上修改而来。

固件在mosaico上做 50 Hz 推理，通过网页摇杆遥控，并通过 UART 把关节角度发给机器人下位机。当前版本屏幕只在开机时刷一帧眼睛，之后不再刷新。

## 环境

- **ESP-IDF 6.1.0**（整个 6.1.x 可以编译）。根目录 `CMakeLists.txt` 在主版本或次版本不是 6.1 时会直接报错；`main/idf_component.yml` 的约束是 `>=6.1.0,<6.2.0`。
- 目标芯片是 **esp32s31**。本工程目前按 V1.2 引脚和面板初始化使用其中与显示、供电相关的部分。

```bash
idf.py set-target esp32s31
idf.py build
idf.py -p PORT flash
```

`PORT` 换成实际串口，例如 Windows 上的 `COM3`。

## 配网

Wi-Fi 名称和密码在 `main/Kconfig.projbuild`：

| 配置项 | 菜单名 | 默认值 |
|---|---|---|
| `DUCK_WIFI_SSID` | Wi-Fi SSID | `XGO2` |
| `DUCK_WIFI_PASSWORD` | Wi-Fi password | `LuwuDynamics` |


网页端口是同一菜单里的 `DUCK_WEB_PORT`，默认 80。

## 查看 IP

设备连上热点后，用下面任一方式看地址。

**串口。** `main/app_main.c` 的 `ip_report_task` 使用 UART0，115200、8N1，每秒输出一行：

```text
WIFI=CONNECTED DUCK_IP=192.168.x.x URL=http://192.168.x.x:80/
```

尚未连上时输出 `WIFI=CONNECTING DUCK_IP=0.0.0.0`。默认 TX 为 GPIO15、RX 为 GPIO16，在 menuconfig 的 `DUCK_IP_UART_TX` / `DUCK_IP_UART_RX` 修改。USB 转串口的 RX 接 GPIO15，GND 共地，电平 3.3 V。这条串口只播报 IP，不发送机器人报文。

**USB 日志。** 同一条内容同时用 `ESP_LOGI` 打到 USB Serial/JTAG。烧录用的 Type-C 上执行 `idf.py -p COMx monitor`，大约每秒一行 `s31_duck: WIFI=CONNECTED DUCK_IP=...`。

**网页。** 手机或电脑连到同一网络，打开串口给出的 URL。页面状态栏里的 `IP` 来自 `GET /api/status` 的 `ip` 字段。

## 使用

上电后机器人保持默认构型。打开网页，等下位机反馈就绪，再点「开始推理」。左摇杆控制前后和左右，右摇杆转向，四个滑条控制头颈。点「返回默认构型」、关闭页面、页面进入后台、Wi-Fi 断开或反馈超时后，推理停止并回到默认构型；重新连上后需要再点一次开始。

下位机接 UART1，默认 TX GPIO46、RX GPIO39（`DUCK_ROBOT_UART_TX` / `DUCK_ROBOT_UART_RX`）。固件先以 115200 要求对端切到 2,000,000 baud，再收状态、发角度。应用日志走 USB Serial/JTAG，不要和上面两条 UART 混用。

策略使用下位机回传的 IMU、关节角度和关节速度。板载 BMI270不进入策略。
