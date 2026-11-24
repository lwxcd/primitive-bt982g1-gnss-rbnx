# Driver CMD_INIT configuration; independent of body/URDF and external ROS workspaces.
# serial.port: auto (default) or an absolute device path. auto tries /dev/rtk first,
# then a unique USB serial device matching usb_vid/usb_pid.
# serial.baud: positive integer, default 115200; must match the receiver.
# serial.usb_vid / usb_pid: optional integers [0,65535], supplied together and only
# with port=auto. Defaults identify the PL2303G bridge (0x067B / 0x23A3).
# serial.init_commands: printable ASCII commands written with CRLF on every open;
# default ["MODE ROVER"]. Set [] to leave the receiver configuration untouched.
# ntrip.enabled: bool, default true. With incomplete credentials the receiver keeps
# running on single-point GNSS; NTRIP state reports disabled.
# ntrip.env_file: env file path, default ~/.config/rtk_imu/ntrip.env. Reads
# NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT, NTRIP_USER, NTRIP_PASS or NTRIP_PASS_FILE,
# NTRIP_VERSION, NTRIP_TLS, NTRIP_PROXY, NTRIP_HOST_HEADER.
# ntrip.reconnect_s: number (0,300], default 10.0.
# ntrip.gga_interval_s: number (0,120], default 10.0; VRS GGA uplink period.
# frame_id: nonempty string without whitespace, default gnss_link; no TF is published.
# status_rate_hz: number (0,20], default 1.0.
# sentinel_timeout_s: number (0,120], default 30.0. ACTIVATE fails unless a valid
# NavSatFix and a status sample with receiver data arrive before the deadline.
# require_velocity: boolean, default false. When true ACTIVATE also waits for a
# finite ENU /gps/velocity sample. Enable this for the localization service.
# velocity_source: rmc (default), position_diff or auto. rmc uses the receiver's
# RMC ground speed and true track. position_diff derives ENU velocity from two
# RTK-fixed GGA positions 0.5-1.5 s apart (closest to 1.0 s) and publishes
# nothing when no such baseline exists, never falling back to RMC. auto uses
# position_diff while it is fresh on a fixed solution and falls back to RMC
# otherwise. Position differencing needs GGA at >= 1 Hz; 2 Hz is recommended.
# topics: optional absolute ROS topic overrides, all must be distinct:
#   fix: /gps/fix
#   rtk_status: /gps/rtk_status
#   velocity: /gps/velocity
# Restart the provider to change already declared topic endpoints.
# Position covariance and the default velocity are receiver-reported only: RMC
# true-track ground speed and GST sigmas must already be enabled on the receiver
# (for example init_commands: ["MODE ROVER", "GNGGA 1", "GNRMC 1", "GNGST 1"]).
# Missing RMC/GST degrades to no velocity / unknown covariance; covariance is
# never fabricated from HDOP, and the optional position_diff velocity is only
# derived from two RTK-fixed positions (neither source reports a sigma).
