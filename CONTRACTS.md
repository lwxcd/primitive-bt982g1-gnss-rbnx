# BT-982G1 GNSS 能力约定 v1

框架当前没有全局 GNSS 约定，本包在 `robonix/primitive/gnss` 下新增三条局部
`topic_out` 约定，均通过 ROS 2 传输，默认 QoS 为 best-effort / volatile /
keep_last(10)。IDL 复用框架 `common_interfaces` 中的
`sensor_msgs/msg/NavSatFix`、`std_msgs/msg/String` 和
`geometry_msgs/msg/TwistStamped`，不引入新消息包或 RPC。

| 约定 | 类型 | 固定端点 |
| --- | --- | --- |
| `robonix/primitive/gnss/fix` | `sensor_msgs/msg/NavSatFix` | `/gps/fix` |
| `robonix/primitive/gnss/rtk_status` | `std_msgs/msg/String` | `/gps/rtk_status` |
| `robonix/primitive/gnss/velocity` | `geometry_msgs/msg/TwistStamped` | `/gps/velocity` |

## 位置 fix

`robonix/primitive/gnss/fix` 的类型是 `sensor_msgs/msg/NavSatFix`。

- 只在 GGA 同时给出有效经纬度且 quality > 0 时发布；无定位时不发布空坐标。
- `header.stamp` 是主机串口接收并发布该帧的时刻，不是接收机内部采样时刻。
- `header.frame_id` 来自配置 `frame_id`（默认 `gnss_link`）。本包不发布 TF。
- quality ∈ {2,4,5} 时 `status.status = STATUS_GBAS_FIX`，其余置 `STATUS_FIX`；
  `status.service = SERVICE_GPS (1)` 是常规占位，接收机实际为多星座，消费方不应
  据此推断星座组成。`rtk_status.state` 才是固定/浮点解的依据。
- `altitude` 缺失时填 `0.0`（NavSatFix 无 null）；此时以 `rtk_status.alt_m = null`
  为准。高度为 GGA 的大地高（字段 9）。
- 同 UTC 的 GST 到达后，用接收机给出的纬度/经度/高度标准差平方填充
  `position_covariance[4]/[0]/[8]`，`position_covariance_type =
  COVARIANCE_TYPE_DIAGONAL_KNOWN`，并重发**同一 header stamp**（GST 后到不是新定位）。
  没有有效 GST 时保持全零 + `COVARIANCE_TYPE_UNKNOWN`，表示未知协方差而不是零误差；
  绝不从 HDOP 推算。定位服务要求 `position_covariance_type != 0` 且 cov[0]/cov[4] 为正。

## 状态 rtk_status

`robonix/primitive/gnss/rtk_status` 的 `String.data` 是 UTF-8 JSON 对象，允许将来增加
字段；当前约定：

| 字段 | 类型 | 语义 |
| --- | --- | --- |
| timestamp_source | string | 恒为 `HOST_SERIAL_RECEIVE` |
| state | string | `NO_DATA`、`STALE`、`NO_FIX`、`SINGLE`、`DGPS`、`RTK_FIXED`、`RTK_FLOAT`、`DEAD_RECKONING`、`UNKNOWN_QUALITY`；消费方应容忍未知值 |
| rtk_fixed | boolean | `state == RTK_FIXED` 且当前 `valid` 为真 |
| serial_connected | boolean | 当前是否持有已打开串口 |
| serial_state / serial_status | string | `OPEN`/`DISCONNECTED` 与 `closed`/`open`/`open failed: ...` |
| serial_error | string | 串口未打开时的诊断文本，否则为空串 |
| port / baud | string / integer | 实际解析的设备路径与波特率 |
| frame_id | string | 与 fix 一致 |
| receiver_quality | integer/null | GGA 字段 6：0 无定位、1 单点、2 DGPS、4 RTK 固定、5 RTK 浮点、6 航位推算 |
| receiver_quality_name | string/null | 上述取值的名称 |
| fix_age_s | number/null | 距最近一次 quality>0 且坐标有效的 GGA 的秒数；从未定位为 null |
| gga_age_s | number/null | 距最近一条解析成功 GGA 的秒数（不论 quality），用于区分 `NO_FIX` 与 `STALE` |
| measurement_age_s | number/null | 距最近一条校验通过的 NMEA 测量（GGA/RMC/VTG/GST/THS/TRA）的秒数；从未收到为 null |
| fix_stamp_s | number/null | 最近一次发布 NavSatFix 的 header stamp（秒，未四舍五入），用于与 fix 历史精确关联 |
| utc_time | string/null | 最近 GGA 原始 UTC（hhmmss.ss） |
| valid | boolean | 串口已打开、quality>0、坐标有效且 `fix_age_s <= 2.0` |
| lat / lon | number/null | WGS84 十进制度；无有效 GGA 时为 null |
| alt_m | number/null | GGA 大地高（m） |
| sats / satellites | integer/null | GGA 卫星数（两个字段同值，`satellites` 为 G70 兼容名） |
| hdop | number/null | GGA HDOP；不得用于推算协方差 |
| diff_age_s / differential_age_s | number/null | GGA 差分龄期（两个字段同值，后者为 G70 兼容名） |
| speed_mps / course_deg | number/null | 最近 RMC/VTG 的接收机地速与航迹角（诊断）；无有效解为 null |
| velocity | object | 见下节；始终存在，无速度时各值为 null/false |
| position_covariance_source | string | `NMEA_GST`（有同 UTC 新鲜 GST）或 `UNKNOWN` |
| heading_deg / heading_pitch_deg | number/null | 双天线 THS/TRA 航向（相对真北）；超过 1.5 s 未更新则为 null |
| heading_source | string/null | `THS` 或 `TRA` |
| counts | object | gga、rmc、vtg、gst、ths、tra、other 累计句数 |
| checksum_errors / bad_nmea | integer | 校验失败累计数；重连后保留计数（两字段同值） |
| ntrip | object | `state`（disabled/down/up/auth-error）、`connects`、`rtcm_bytes`、`rtcm_dropped`、`gga_uploaded`、`last_error` |

`state` 按最近一条 GGA 的新鲜度（`gga_age_s <= 2.0`）计算；串口断开或超时一律
`STALE`，从未收到 GGA 为 `NO_DATA`。`rtk_fixed` 不能代替消费方对差分龄期、
坐标有效性和时间一致性的判断。

### 速度 velocity 与状态内 velocity 对象

`robonix/primitive/gnss/velocity` 的类型是 `geometry_msgs/msg/TwistStamped`：

- `header.frame_id` 恒为 `gnss_enu`（局部 ENU），`linear.x = 东向速度`、
  `linear.y = 北向速度`，单位 m/s；`linear.z`/角速度不填。
- 默认（`velocity_source: rmc`）来源是校验通过的 RMC 地速与真航迹角
  （`speed * sin/cos(course)`），仅在新 UTC 的 RMC 到达时发布；相同 UTC 的重复
  RMC 不刷新测量时间。不声称必为多普勒来源，不读取控制指令。
- `velocity_source: position_diff` 时改用连续两条 RTK 固定解 GGA 的位置差计算
  ENU 速度：基线取最近 0.5–1.5 s 内最接近 1.0 s 的一条（目标 1.0 s），两端都必须是
  quality 4，速度超过 10 m/s 视为异常并丢弃；满足条件时随新 GGA 发布（当前接收机
  配置 `GPGGA 1`，即 1 Hz；提高 GGA 速率可降低延迟，基线仍取目标 1.0 s）。
  不满足条件时**不发布**速度，也不回退 RMC。`auto` 在固定解且位置差速度新鲜时用
  位置差，否则回退 RMC。位置差速度的 `utc_time` 就是触发它的那条 GGA 的 UTC，
  与 `rtk_status.utc_time`/`fix` 的 UTC 天然一致。
- `header.stamp` 是主机发布该消息的时刻；`rtk_status.velocity.stamp_s` 与它精确
  一致，供消费方按 stamp 从历史中取回对应速度消息。
- RMC 路径下 RMC 无效（status != A、mode=N、非有限值、有速度无航迹）时清空速度并
  停止刷新；没有 RMC 时不发布速度。

`rtk_status.velocity` 字段（无速度时仍存在）：

| 字段 | 类型 | 语义 |
| --- | --- | --- |
| source | string | `NMEA_RMC` 或 `POSITION_DIFF`；无速度样本时给出配置的首选源（`auto` 报 `NMEA_RMC`） |
| fresh | boolean | 最近速度在 2.0 s 内且有效 |
| age_s | number/null | 距最近有效速度样本的秒数 |
| utc_time | string/null | 该速度样本的 UTC（hhmmss.ss）；位置差分为触发它的 GGA 的 UTC |
| stamp_s | number/null | 对应 `/gps/velocity` 的 header stamp（秒，未四舍五入） |
| reference_frame | string | `ENU` |
| covariance_known | boolean | 恒为 `false`；RMC 与位置差都不报告速度不确定度 |

定位服务要求：位置与速度的 UTC 相差不超过 0.001 s（除非使用 `epoch_source =
UBX_iTOW_ms`，本包不走该路径）；`state ∈ {RTK_FIXED, RTK_FLOAT}`；协方差已知；
并且要么有接收机报告的速度精度（`accuracy_source = UBX_NAV_PVT`，本包不提供），
要么部署方显式设置 `velocity_accuracy_confirmed` 与 `velocity_accuracy_mps`。
因此本包接入定位服务时必须由部署方完成速度精度确认，不能把 RMC 或位置差速度当作
已验证精度。

### 新鲜度与消费方约束

- 状态按 `status_rate_hz`（默认 1 Hz）周期发布；fix 按接收机 GGA 速率发布，
  同 UTC 的 GST 只触发同 stamp 重发，不增加新定位。
- `fix_age_s`、`gga_age_s` 与 `measurement_age_s` 在每条状态消息构造时计算，
  不是接收机内部时间。
- **室外建图服务** `outdoor_mapping` 要求：fix 与 rtk_status 均在其
  `input_timeout_s`（默认 2.0 s）内收到，`receiver_quality ∈ {2,4,5}`，且
  `fix_age_s`/`measurement_age_s` 同为不超过该超时的非负数值。本原语不替消费方做
  该质量门，原样上报接收机结果。
- **室外定位服务** `service-localization-outdoor-rbnx` 通过 Atlas 解析
  `fix`、`rtk_status`、`velocity` 三条契约；它按 `fix_stamp_s`/`velocity.stamp_s`
  精确匹配历史消息，再检查 state、两个龄期、协方差、ENU frame 与 UTC 对齐。
  本原语只提供接收机测量，不提供定位、TF 或融合。定位服务默认
  `sensor_timeout_s = 1.0`；本原语默认 NMEA 输出为 1 Hz，fix 与速度消息在定位
  服务历史缓存中的最老年龄可接近 1 s，状态自身还有发布间隔，边界上会出现周期性
  `gnss_stream_stale`。部署时应把定位服务的 `sensor_timeout_s` 放宽到至少 2.0 s，
  或按 Unicore 文档提高接收机 GGA/RMC 输出率；同时建议 `status_rate_hz: 2.0`，
  避免状态刷新间隔单独吃掉超时预算。
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
