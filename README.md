# BT-982G1 GNSS/RTK 原语

把 Unicore BT-982G1（UM982 协议）双天线接收机接入 Robonix。原语独占一个串口，
解析 NMEA GGA/RMC/VTG 与双天线 THS/TRA，可选通过 NTRIP 注入 RTCM 差分改正，
发布标准 `robonix/primitive/gnss/fix` 与包内定义的 `rtk_status`。
NMEA/NTRIP 逻辑取自本仓库原有的 `scripts/rtk_imu/{rover_status,ntrip_client}.py`
台架脚本，去掉了 IMU 串口、网页仪表盘和 CSV 记录。

## 与室外服务的关系

- `service-map-outdoor-rbnx`（`outdoor_mapping`）通过 `gnss_provider_id` 显式选择
  本实例，消费 `fix` 与 `rtk_status`；它要求 `receiver_quality ∈ {2,4,5}`
  且两个龄期不超过其 `input_timeout_s`（默认 2 s）。
- `service-navigation-outdoor-rbnx`（`outdoor_execution`）**不直接消费 GNSS**。
  它需要独立定位服务提供 `robonix/service/localization/{odom,status}`；本原语
  不能替代该服务，也不能单独满足导航执行的生命周期依赖。
- 与 WIT IMU 原语互不冲突：WIT 使用 CP2102 串口和 9600 波特率，本接收机使用
  PL2303G 的 `/dev/rtk` 和 115200 波特率。

## 环境与构建

Linux、ROS 2（Humble 或更新）、rbnx/robonix-api；Python 可导入 `rclpy`、`serial`、
`grpc`。ROS 环境提供 `sensor_msgs`、`std_msgs`。运行用户需有串口读写权限
（通常加入 `dialout` 组）。不需要编译自定义 ROS 消息或 colcon 工作空间。

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd /path/to/primitive-bt982g1-gnss-rbnx
rbnx validate .
bash scripts/build.sh
```

`start.sh` 使用 `ROS_SETUP` 指定的 setup.bash，或 `/opt/ros/$ROS_DISTRO/setup.bash`
（未指定发行版时尝试 humble），也支持已 source 的其他 ROS 安装。
`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、Zenoh 配置由部署环境传入。
`BT982G1_PYTHON` 可指定可导入 ROS 2 及 Robonix 依赖的解释器，默认 `/usr/bin/python3`。

## 加入部署

```yaml
primitive:
  - name: bt982g1_gnss
    path: ../lite3/primitive-bt982g1-gnss-rbnx
    config:
      serial:
        port: /dev/rtk
        baud: 115200
        init_commands: ["MODE ROVER"]
      ntrip:
        enabled: true
        env_file: "~/.config/rtk_imu/ntrip.env"
      frame_id: gnss_link
      status_rate_hz: 1.0
      sentinel_timeout_s: 30.0
```

实例身份来自 `RBNX_INSTANCE_NAME`，默认 `bt982g1_gnss`，必须与
`outdoor_mapping.config.gnss_provider_id` 一致。

**接收机未连接时不要启用该条目。** ACTIVATE 会等待真实 fix 与状态样本，超时返回
错误；Soma 把任一原语激活失败视为致命并中止整个 boot。硬件拔掉期间应把整段
primitive 条目注释掉（空挂），恢复后取消注释并重启整栈。

## 配置

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `serial.port` | `auto` | `auto` 先找 `/dev/rtk`，再按 USB VID/PID 唯一匹配；也可写绝对路径 |
| `serial.baud` | `115200` | 必须与接收机一致 |
| `serial.usb_vid` / `usb_pid` | `0x067B` / `0x23A3` | 仅 `port: auto` 使用，两项须同时给出 |
| `serial.init_commands` | `["MODE ROVER"]` | 每次打开串口后按 CRLF 发送；`[]` 表示不配置接收机 |
| `ntrip.enabled` | `true` | 凭据不全时只跑单点，不阻塞激活 |
| `ntrip.env_file` | `~/.config/rtk_imu/ntrip.env` | NTRIP 凭据文件 |
| `ntrip.reconnect_s` | `10.0` | 断线重连间隔 |
| `ntrip.gga_interval_s` | `10.0` | VRS 挂载点的 GGA 上行周期 |
| `frame_id` | `gnss_link` | fix 的坐标系；本包不发布 TF |
| `status_rate_hz` | `1.0` | 状态 JSON 发布频率，范围 (0,20] |
| `sentinel_timeout_s` | `30.0` | 激活时等待首个有效 fix 与状态的上限 |
| `topics.fix` | `/gps/fix` | NavSatFix |
| `topics.rtk_status` | `/gps/rtk_status` | JSON String |

完整语义见 `CONTRACTS.md`。

## NTRIP

凭据只从 env 文件读取（避免出现在命令行与日志中）：

```bash
NTRIP_HOST=caster.example.com
NTRIP_PORT=2101
NTRIP_MOUNT=RTCM32_GGB
NTRIP_USER=account
NTRIP_PASS=secret
NTRIP_VERSION=2
NTRIP_TLS=0
NTRIP_PROXY=            # 例如 socks5://127.0.0.5:10808
NTRIP_HOST_HEADER=
```

也可用 `NTRIP_PASS_FILE=/path/to/pass` 代替 `NTRIP_PASS`。文件缺失或字段不全时
原语保持运行，`rtk_status.ntrip.state = disabled`，定位等级由接收机自身决定。
认证被拒时状态为 `auth-error`，每 30 s 重试；其他错误按 `reconnect_s` 重试。

## 硬件与串口

- 设备识别：`lsusb -d 067b:23a3`。若能看到设备但没有 `/dev/rtk`，需先安装
  PL2303G DKMS 驱动（见 `scripts/rover-package/`），然后重新登录使 `dialout` 组生效。
- `/dev/rtk` 是推荐路径；没有 udev 规则时可用 `auto` 按 USB ID 匹配，或填写
  `/dev/serial/by-id/...` 稳定路径。
- **串口独占**：激活前停止 `rover_status.py`、`rtk_imu_fusion.py`、`rtk_test.py`
  等占用同一串口的脚本。驱动打开端口后这些脚本无法再读取。
- 默认在每次打开串口时发送 `MODE ROVER`；若接收机已预先配置，可设
  `init_commands: []`，避免覆盖既有设置。

## 话题与生命周期

- `fix`：仅在 quality>0 且经纬度有效时发布，`status.status = 0`，协方差未知。
- `rtk_status`：周期发布 JSON，字段包括 `receiver_quality`、`fix_age_s`、
  `measurement_age_s`、`valid`、卫星/HDOP/差分龄期、速度/航迹角、双天线航向
  （仅诊断）、NTRIP 计数器。
- ACTIVATE 期间原语订阅自己发布的两个话题，必须同时收到有效 NavSatFix 和含实际
  测量龄期的状态才注册能力；否则返回错误并清理串口与线程。
- DEACTIVATE/SHUTDOWN 停止 NTRIP 客户端、接收线程和状态线程，关闭串口与 rclpy
  context。
- 不发布 TF、odom 或独立航向能力。

## 测试

```bash
cd /path/to/primitive-bt982g1-gnss-rbnx
PYTHONPATH=. python3 -m pytest tests -q
```

单元测试覆盖 NMEA 校验和/坐标/各语句解析、配置校验、NTRIP 凭据与代理解析；
不需要接收机或 ROS。实机验证需要连接接收机、天线和（可选）NTRIP 网络，检查
`ros2 topic echo /gps/fix --once` 与 `/gps/rtk_status` 的等级和龄期。
