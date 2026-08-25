# nus2217 — RFC2217 网络串口代理

把 BLE 桥暴露为 RFC2217 网络串口，esptool / PlatformIO / 任何 pyserial 程序可以原生使用，
包括 DTR/RTS 复位和波特率切换（分别映射到 `6E400004` / `6E400005` 特征）。

## 启动

```bash
# 依赖 uv（脚本头内置依赖声明：bleak + pyserial）
./tools/nus2217.py                 # 默认按名称 ESP32_BRIDGE 扫描，监听 localhost:4000
./tools/nus2217.py -d <BLE地址>    # 指定设备地址
./tools/nus2217.py -p 4001 -v      # 自定义端口 / 调试日志
```

## esptool / PlatformIO：原生可用

原版 esptool 和 `pio run -t upload` 都可以直接工作，包括 stub flasher：

```bash
esptool --port rfc2217://localhost:4000 read-flash-status
esptool --port rfc2217://localhost:4000 write-flash 0x10000 app.bin

# PlatformIO（实测 1.7MB 固件 65 秒完成，448 kbit/s）
pio run -t upload --upload-port rfc2217://localhost:4000
```

或写入 `platformio.ini`：

```ini
upload_port  = rfc2217://localhost:4000
monitor_port = rfc2217://localhost:4000
```

## 实现说明

- **sync 应答补齐（固件内置）**：ESP ROM 对一次 SYNC 回 8 个连发应答，桥的 USB 主机
  （单 URB 软件轮询）赶不上 ROM 的 USB 写超时，尾部应答会丢。桥固件识别 SYNC 交换后
  把应答补齐到 8 个（内容全等，客户端无法区分），因此任何直连 BLE 的 esptool 客户端
  （包括 iPad 端实现）都无需自行处理。
- **AcquireWrite**：代理自动尝试 BlueZ 的 fd 直写路径（绕过每次写 10–30ms 的 D-Bus 往返），
  不可用时回退 D-Bus 写。
- **数据通道纯透传**：固件不往数据特征注入任何状态文本（曾经的 `>> Serial Device connected`
  等提示会破坏 esptool 的 SLIP 流，已移除）。

## Linux 内核调优（建议，重启后失效）

内核中心端默认连接间隔 30–50ms，调到 7.5–15ms 可显著降低延迟：

```bash
echo 6  | sudo tee /sys/kernel/debug/bluetooth/hci0/conn_min_interval
echo 12 | sudo tee /sys/kernel/debug/bluetooth/hci0/conn_max_interval
```

## 已知问题

- **异常断连后 notify 订阅失败**（`Failed to register notify session` / `Unlikely Error`）：
  Bluedroid 的残留状态问题，重启桥板恢复；主机端 `bluetoothctl remove <地址>` 清 BlueZ 缓存。
- 一次只接受一个 TCP 客户端；BLE 断开时代理退出（重新运行即可）。
- 吞吐参考：写方向 ~450 kbit/s（pio 烧录实测），读方向 ~200 kbit/s（flash 读取实测）。
