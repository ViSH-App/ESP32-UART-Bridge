# iPad 接入文档

ESP32-S3 VCP-BT Bridge 把一个 USB 串口设备（CP210x / CH34x / FTDI）桥接为 BLE 串口。固件实现了 **Nordic UART Service（NUS）**——BLE 串口的事实标准——因此 iPad 上既可以用现成的 NUS 终端 App 直接收发，也可以用 CoreBluetooth 自行开发。

## 设备信息

| 项目 | 值 |
|---|---|
| 广播名称 | `ESP32_BRIDGE` |
| 服务 UUID | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`（Nordic UART Service） |
| RX 特征（iPad → 设备） | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`，支持 Write 和 Write Without Response |
| TX 特征（设备 → iPad） | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`，Notify |
| 控制线特征 | `6E400004-B5A3-F393-E0A9-E50E24DCCA9E`，Write / Read，1 字节 |
| 线路参数特征 | `6E400005-B5A3-F393-E0A9-E50E24DCCA9E`，Write / Read，7 字节 |
| 串口状态特征 | `6E400006-B5A3-F393-E0A9-E50E24DCCA9E`，Notify / Read，2 字节 |
| 配对 | 不需要，连接即用 |
| 串口默认参数（USB 侧） | 115200 8N1，可通过线路参数特征动态修改 |

写入 RX 特征的字节会原样发往 USB 串口；USB 串口收到的字节通过 TX 特征的 notify 推送给 iPad。数据是透明字节流，无帧格式、无转义。`0002`/`0003` 是标准 NUS，现成的 NUS 终端 App 可直接使用；`0004`–`0006` 是本设备的串口控制扩展，NUS App 会自动忽略。

## 串口控制特征

### 控制线 `6E400004`（DTR / RTS）

1 字节位掩码，沿用 USB CDC `SET_CONTROL_LINE_STATE` 的定义：

| 位 | 含义 |
|---|---|
| bit0 | DTR（1 = 有效） |
| bit1 | RTS（1 = 有效） |

写入立即生效；Read 返回当前设置。上电默认 `0x00`（均无效）。设置会记忆，USB 串口设备拔插后自动恢复。

### 线路参数 `6E400005`（波特率 / 数据位 / 校验 / 停止位）

7 字节，即 USB CDC Line Coding 结构：

| 偏移 | 长度 | 含义 |
|---|---|---|
| 0 | 4 | 波特率，小端 uint32 |
| 4 | 1 | 停止位：0 = 1 位，1 = 1.5 位，2 = 2 位 |
| 5 | 1 | 校验：0 = 无，1 = 奇，2 = 偶，3 = Mark，4 = Space |
| 6 | 1 | 数据位：5 / 6 / 7 / 8 |

示例：9600 8N1 = `80 25 00 00 00 00 08`。写入立即生效并记忆；重启后恢复默认 115200 8N1。

### 串口状态 `6E400006`（CTS / DSR 等，设备 → iPad）

2 字节小端，即 USB CDC SerialState 位图，串口芯片状态变化时通过 notify 推送：

| 位 | 含义 |
|---|---|
| bit0 | DCD（载波检测） |
| bit1 | DSR |
| bit2 | Break 检测 |
| bit3 | RI（振铃） |
| bit4–6 | 帧错误 / 校验错误 / 溢出 |

订阅方式与 TX 特征相同（`setNotifyValue(true)`）。注意各芯片支持程度不同（FTDI 最全，CH34x 部分支持）。

### 暂不支持

Send Break：ESP-IDF 的 VCP 驱动中只有 CP210x 实现了 break，CH34x 和 FTDI 未实现，故未暴露。

## 重要：设备不会出现在系统蓝牙设置里

iPadOS 的「设置 → 蓝牙」只显示经典蓝牙和特定标准 profile 的设备，不显示通用 BLE GATT 外设。这是 iOS 的设计，不是故障。连接必须由 App 通过 CoreBluetooth 发起。

## 方式一：使用现成 App（无需开发）

以下 App 均原生支持 NUS，扫描到 `ESP32_BRIDGE` 后点击连接即可当串口终端使用：

- **Bluefruit Connect**（Adafruit，免费）— 连接后选 UART 模式
- **nRF Toolbox**（Nordic，免费）— 选 UART 工具
- **nRF Connect**（Nordic，免费）— 通用 GATT 调试器，适合排查问题

连接过程不会弹配对框。断开后设备自动恢复广播，可重新连接。

## 方式二：CoreBluetooth 开发

### 前置条件

- Info.plist 添加 `NSBluetoothAlwaysUsageDescription`（缺失会导致 App 启动蓝牙时崩溃）
- 如需后台保持连接：Signing & Capabilities → Background Modes → 勾选 "Uses Bluetooth LE accessories"

### 完整流程示例

```swift
import CoreBluetooth

enum NUS {
    static let service = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    static let rx      = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E") // iPad 写数据
    static let tx      = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E") // iPad 收数据
    static let ctrl    = CBUUID(string: "6E400004-B5A3-F393-E0A9-E50E24DCCA9E") // DTR/RTS
    static let line    = CBUUID(string: "6E400005-B5A3-F393-E0A9-E50E24DCCA9E") // 波特率等
    static let state   = CBUUID(string: "6E400006-B5A3-F393-E0A9-E50E24DCCA9E") // CTS/DSR 等
}

final class SerialBridge: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?   // 必须强引用，否则连接中途被释放
    private var rxChar: CBCharacteristic?

    var onReceive: ((Data) -> Void)?
    var onReady: (() -> Void)?

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    // 1. 蓝牙就绪后按服务 UUID 扫描（广播包里带 NUS UUID，可直接过滤）
    func centralManagerDidUpdateState(_ c: CBCentralManager) {
        if c.state == .poweredOn {
            c.scanForPeripherals(withServices: [NUS.service])
        }
    }

    // 2. 发现即连接
    func centralManager(_ c: CBCentralManager, didDiscover p: CBPeripheral,
                        advertisementData: [String: Any], rssi: NSNumber) {
        peripheral = p
        c.stopScan()
        c.connect(p)
    }

    // 3. 连接成功 → 发现服务
    func centralManager(_ c: CBCentralManager, didConnect p: CBPeripheral) {
        p.delegate = self
        p.discoverServices([NUS.service])
    }

    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        p.services?.forEach {
            p.discoverCharacteristics([NUS.rx, NUS.tx], for: $0)
        }
    }

    // 4. 拿到特征：订阅 TX，记下 RX
    func peripheral(_ p: CBPeripheral,
                    didDiscoverCharacteristicsFor s: CBService, error: Error?) {
        for ch in s.characteristics ?? [] {
            switch ch.uuid {
            case NUS.rx: rxChar = ch
            case NUS.tx: p.setNotifyValue(true, for: ch)
            default: break
            }
        }
        if rxChar != nil { onReady?() }
    }

    // 5. 收数据（设备 → iPad）
    func peripheral(_ p: CBPeripheral,
                    didUpdateValueFor ch: CBCharacteristic, error: Error?) {
        if let data = ch.value { onReceive?(data) }
    }

    // 6. 发数据（iPad → 设备），按 MTU 自动分片
    func send(_ data: Data) {
        guard let p = peripheral, let ch = rxChar else { return }
        let mtu = p.maximumWriteValueLength(for: .withoutResponse)
        var offset = 0
        while offset < data.count {
            let chunk = data.subdata(in: offset ..< min(offset + mtu, data.count))
            p.writeValue(chunk, for: ch, type: .withoutResponse)
            offset += chunk.count
        }
    }

    // 7. 断线自动重连
    func centralManager(_ c: CBCentralManager, didDisconnectPeripheral p: CBPeripheral,
                        error: Error?) {
        c.connect(p)   // 设备断开后会立即恢复广播
    }
}
```

### 开发要点

**写入方式**：优先用 `.withoutResponse`（吞吐高）。iOS 会自动做发送队列节流；如果要更严谨，在 `peripheralIsReady(toSendWriteWithoutResponse:)` 回调里续发。`.withResponse` 也支持，适合低速率、需要逐包确认的场景。

**分片**：单次 `writeValue` 不能超过 `maximumWriteValueLength(for:)` 返回的长度（iOS 通常协商到 ATT MTU 185 或更高，即约 182 字节）。上面的 `send` 已做分片。

**接收**：设备端已按协商 MTU 分片发送 notify，iPad 侧每次 `didUpdateValueFor` 收到一段字节流。注意 BLE 不保证消息边界与串口写入边界一致——如果上层协议有帧概念，需要自己按协议重组。

**数据是纯透传**：TX 特征上收到的所有字节都是串口数据，没有任何带内控制字节，可安全传输任意二进制。

**数据通道完全透明**：TX 特征上只有串口数据，无任何带内状态文本或控制字节，可安全传输任意二进制协议。USB 串口设备的插拔状态请通过读取控制线/状态特征或数据流本身判断。

**串口控制示例**：

```swift
// 置位 DTR + RTS（如复位桥后面的目标板）
peripheral.writeValue(Data([0x03]), for: ctrlChar, type: .withResponse)

// 切换到 9600 8N1
var lc = Data()
lc.append(contentsOf: withUnsafeBytes(of: UInt32(9600).littleEndian) { Array($0) })
lc.append(contentsOf: [0, 0, 8])  // 停止位、校验、数据位
peripheral.writeValue(lc, for: lineChar, type: .withResponse)

// 订阅 CTS/DSR 状态变化
peripheral.setNotifyValue(true, for: stateChar)
```

控制写建议用 `.withResponse`，写入即生效且可通过 Read 读回确认。控制命令与数据写在设备端按到达顺序执行，因此「先发数据、再改波特率」这类序列是安全的。

## 故障排查

| 现象 | 处理 |
|---|---|
| 扫描不到设备 | 确认设备已上电；确认没有别的客户端占着连接（BLE 连接期间会停止广播，断开后自动恢复）；App 的扫描过滤 UUID 是否为 `6E400001-…` |
| 之前连过旧固件，行为异常 | 旧版固件（`ESP_SPP_SERVER`）会强制配对。到「设置 → 蓝牙」里找到旧条目选「忽略此设备」，或在 nRF Connect 里删除绑定后重试 |
| 能连上但收不到数据 | 确认已对 TX 特征（`6E400003-…`）调用 `setNotifyValue(true)`；确认 USB 侧串口设备已插入（板载 LED 蓝色微亮表示串口已连接） |
| 写入报错 | 单次写入是否超过 `maximumWriteValueLength`；RX 特征 UUID 是否用错（应为 `6E400002-…`） |

## 板载 LED 状态

| 颜色 | 含义 |
|---|---|
| 熄灭 | 无连接 |
| 蓝色微亮 | USB 串口已连接，BLE 未连接 |
| 紫色 | BLE 已连接 |
| 红色闪烁 | 串口 → BLE 方向有数据 |
| 绿色闪烁 | BLE → 串口方向有数据 |
| 黄色闪烁 | 双向同时有数据 |
