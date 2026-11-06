# BT-982G1 GNSS 能力约定 v1

框架当前没有全局 GNSS 约定，本包在 `robonix/primitive/gnss` 下新增两条局部
`topic_out` 约定，均通过 ROS 2 传输，默认 QoS 为 best-effort / volatile /
keep_last(10)。IDL 复用框架 `common_interfaces` 中的
`sensor_msgs/msg/NavSatFix` 和 `std_msgs/msg/String`，不引入新消息包或 RPC。

## 位置 fix

`robonix/primitive/gnss/fix` 的类型是 `sensor_msgs/msg/NavSatFix`。

- 只在 GGA 同时给出有效经纬度且 quality > 0 时发布；无定位时不发布空坐标。
- `header.stamp` 是主机串口接收并发布该帧的时刻，不是接收机内部采样时刻。
- `header.frame_id` 来自配置 `frame_id`（默认 `gnss_link`）。本包不发布 TF。
- `status.status = STATUS_FIX (0)`；`status.service = SERVICE_GPS (1)` 是常规占位，
  接收机实际为多星座，消费方不应据此推断星座组成。
- `altitude` 缺失时填 `0.0`（NavSatFix 无 null）；此时以 `rtk_status.alt_m = null`
  为准。高度为 GGA 的大地高（字段 9）。
- `position_covariance` 全零、`position_covariance_type = COVARIANCE_TYPE_UNKNOWN`，
  表示未知协方差，不是零误差；需要精度请使用 `rtk_status` 的 HDOP、卫星数和差分龄期。

## 状态 rtk_status

`robonix/primitive/gnss/rtk_status` 的 `String.data` 是 UTF-8 JSON 对象，允许将来增加
字段；当前约定：

| 字段 | 类型 | 语义 |
| --- | --- | --- |
| timestamp_source | string | 恒为 `HOST_SERIAL_RECEIVE` |
| serial_connected | boolean | 当前是否持有已打开串口 |
| serial_status | string | `closed`、`open` 或 `open failed: ...` |
| port / baud | string / integer | 实际解析的设备路径与波特率 |
| frame_id | string | 与 fix 一致 |
| receiver_quality | integer/null | GGA 字段 6：0 无定位、1 单点、2 DGPS、4 RTK 固定、5 RTK 浮点、6 航位推算 |
| receiver_quality_name | string/null | 上述取值的名称 |
| fix_age_s | number/null | 距最近一次 quality>0 且坐标有效的 GGA 的秒数；从未定位为 null |
| measurement_age_s | number/null | 距最近一条校验通过的 NMEA 测量（GGA/RMC/VTG/THS/TRA）的秒数；从未收到为 null |
| valid | boolean | 串口已打开、quality>0、坐标有效且 `fix_age_s <= 2.0` |
| lat / lon | number/null | WGS84 十进制度；无有效 GGA 时为 null |
| alt_m | number/null | GGA 大地高（m） |
| sats / hdop / diff_age_s | integer / number / number，均可为 null | GGA 卫星数、HDOP、差分龄期 |
| speed_mps / course_deg | number/null | 最近 RMC/VTG 的多普勒速度与航迹角；无有效解为 null |
| heading_deg / heading_pitch_deg | number/null | 双天线 THS/TRA 航向（相对真北）；超过 1.5 s 未更新则为 null |
| heading_source | string/null | `THS` 或 `TRA` |
| counts | object | gga、rmc、vtg、ths、tra、other 累计句数 |
| checksum_errors | integer | 校验失败累计数；重连后保留计数 |
| ntrip | object | `state`（disabled/down/up/auth-error）、`connects`、`rtcm_bytes`、`rtcm_dropped`、`gga_uploaded`、`last_error` |

### 新鲜度与消费方约束

- 状态按 `status_rate_hz`（默认 1 Hz）周期发布；fix 按接收机 GGA 速率发布。
- `fix_age_s` 与 `measurement_age_s` 在每条状态消息构造时计算，不是接收机内部时间。
- **室外建图服务** `outdoor_mapping` 要求：fix 与 rtk_status 均在其
  `input_timeout_s`（默认 2.0 s）内收到，`receiver_quality ∈ {2,4,5}`，且
  `fix_age_s`/`measurement_age_s` 同为不超过该超时的非负数值。本原语不替消费方做
  该质量门，原样上报接收机结果。
- 话题存在或有发布者不等于定位有效；必须同时检查 `receiver_quality`、两个龄期、
  NavSatFix 的 `status.status` 与坐标范围。
- `valid` 是位置有效性的快捷汇总，不表示 RTK 固定，也不表示航向或速度有效。

### 航向边界

双天线 THS/TRA 航向仅放在状态 JSON 中作诊断，用来检查定向天线与安装偏置。
它不是独立能力，不发布 TF/odom，也未标定到车体坐标系；导航定位服务需要独立
约定并显式处理安装旋转，不能直接拿 `heading_deg` 当 `map -> odom` 航向。

### 凭据边界

NTRIP 账号与密码只从 `ntrip.env_file`（或 `NTRIP_PASS_FILE`）读取，不写入话题、
日志或命令行。`last_error` 只包含连接错误或 HTTP 状态，不含凭据。状态 JSON 会
披露 caster 主机与挂载点，按需限制话题可见性。
