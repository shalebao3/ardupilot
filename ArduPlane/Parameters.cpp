#include "Plane.h"

#include <AP_Gripper/AP_Gripper.h>

/*
 *  ArduPlane 参数定义
 *
 */

const AP_Param::Info Plane::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: EEPROM 格式版本号
    // @Description: 当 EEPROM 格式发生变更时递增
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // SYSID_THISMAV was here

    // SYSID_MYGCS was here

    // AP_SerialManager was here

    // @Param: AUTOTUNE_LEVEL
    // @DisplayName: 自动调参等级
    // @Description: 俯仰与横滚 PID 增益的激进程度。数值越小调参越“柔和”。大多数机型推荐 6。为 0 表示保持控制器的 RMAX/TCONST，仅调整 PID 值
    // @Range: 0 10
    // @Increment: 1
    // @User: Standard
    ASCALAR(autotune_level, "AUTOTUNE_LEVEL",  6),

    // @Param: AUTOTUNE_OPTIONS
    // @DisplayName: 自动调参选项位掩码
    // @Description: 固定翼自动调参选项。在 QuadPlane 且 INS_GYRO_FILTER 较高时，可防止固定翼自动调参将滤波值设置过激。
    // @Bitmask: 0: 禁止 Autotune 更新 FLTD
    // @Bitmask: 1: 禁止 Autotune 更新 FLTT
    // @User: Advanced
    ASCALAR(autotune_options, "AUTOTUNE_OPTIONS",  0),

    // TELEM_DELAY was here

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID 调参掩码
    // @Description: 发送 MAVLink PID_TUNING 消息的 PID 位掩码
    // @User: Advanced
    // @Bitmask: 0:横滚,1:俯仰,2:偏航,3:转向,4:降落,5:Z 轴加速度
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: KFF_RDDRMIX
    // @DisplayName: 方向舵混控
    // @Description: 副翼动作时增加的方向舵量。若机头初始偏航与横滚相反，可适当增大以减少不利偏航。
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_THR2PTCH
    // @DisplayName: 油门到俯仰混控
    // @Description: 按油门比例增加抬头俯仰角。100% 油门会在俯仰目标上增加该角度。
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   0),

    // @Param: STAB_PITCH_DOWN
    // @DisplayName: 低油门俯仰下压配平
    // @Description: 在 FBWA/AUTOTUNE 中油门低于 TRIM_THROTTLE 时添加下俯仰角。线性缩放至 THR_MIN 时达到全值，有助于保持滑翔/进近空速并避免失速。多数机型推荐 2 度。
    // @Range: 0 15
    // @Increment: 0.1
    // @Units: deg
    // @User: Advanced
    GSCALAR(stab_pitch_down, "STAB_PITCH_DOWN",   2.0f),

    // @Param: ALT_SLOPE_MIN
    // @DisplayName: 高度坡度最小值
    // @Description: 该值决定在航点高度变化达到多少时采用高度坡度而非立即变高。默认 15m 用于平滑航点附近的小幅高度变化。设为 0 则禁用高度坡度，否则表示触发坡度所需的最小高度误差（m）。
    // @Range: 0 1000
    // @Increment: 1
    // @Units: m
    // @User: Advanced
    GSCALAR(alt_slope_min, "ALT_SLOPE_MIN", 15),

    // @Param: ALT_SLOPE_MAXHGT
    // @DisplayName: 高度坡度最大高度
    // @Description: 飞机可偏离高度坡度的最大高度，超过则重建坡度。用于平滑自动起飞。
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Advanced
    GSCALAR(alt_slope_max_height, "ALT_SLOPE_MAXHGT", 5.0),

    // @Param: STICK_MIXING
    // @DisplayName: 摇杆混控
    // @Description: 启用后在自动模式下将用户摇杆输入叠加到舵面，允许不切模式的有限人工控制。支持 3 种混控：1/4 为电传式混控，4 提供横滚+偏航，1 还提供 FBW-A 俯仰控制；3 仅在 QuadPlane 模式下作用于偏航（如 VTOL 自动起降）。警告：FBW-A 俯仰不含包线保护，模式 1 长时间俯仰输入可能失速或超速。
    // @Values: 0:禁用,1:FBW 风格,3:仅 VTOL 偏航,4:FBW 风格（无俯仰）
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   uint8_t(StickMixing::FBW)),

    // @Param: TKOFF_THR_MINSPD
    // @DisplayName: 起飞油门最小速度
    // @Description: 自动起飞中解除油门抑制的速度检查所用的最小 GPS 地速（m/s）。可用于弹射起飞以确保离开弹射器后再启机，但考虑 GPS 测量误差，更推荐使用 TKOFF_THR_MINACC 与 TKOFF_THR_DELAY。对后推式手抛起飞，建议该值不低于 4 m/s，以避免电机过早启动。注意 GPS 速度会滞后真实速度约 0.5 秒。地速检查由 TKOFF_THR_DELAY 参数延迟。
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_throttle_min_speed,     "TKOFF_THR_MINSPD",  0),

    // @Param: TKOFF_THR_MINACC
    // @DisplayName: 起飞油门最小加速度
    // @Description: 自动起飞中启用地速检查前所需的最小前向加速度（m/s/s），用于手抛起飞。设为 0 将禁用加速度检测，使地速检查始终启用，可能因 GPS 速度跳变而启动电机。手抛与弹弓起飞建议设为约 15。另见 TKOFF_ACCEL_CNT 用于完整“晃动解锁”控制。
    // @Units: m/s/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_throttle_min_accel,     "TKOFF_THR_MINACC",  0),

    // @Param: TKOFF_THR_DELAY
    // @DisplayName: 起飞油门延迟
    // @Description: 在 TKOFF_THR_MINACC 通过后，地速检查延迟的时间（1/10 秒）。对后推式手抛起飞，该值必须不小于 2（0.2 秒），以确保机体离手后电机才启动。弹弓起飞可用更大值（如 30），给弹弓释放留出时间。
    // @Units: ds
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(takeoff_throttle_delay,     "TKOFF_THR_DELAY",  2),

    // @Param: TKOFF_THR_MAX_T
    // @DisplayName: 起飞最大油门时间
    // @Description: 固定翼起飞阶段强制最大油门的持续时间。
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Standard
    ASCALAR(takeoff_throttle_max_t,     "TKOFF_THR_MAX_T",  4),

    // @Param: TKOFF_THR_MIN
    // @DisplayName: 起飞最低油门
    // @Description: 当 TKOFF_OPTIONS 置位 0 位时，在 AUTO/TAKEOFF 起飞所用的最小油门；也用于 QuadPlane 前向转换的最小油门。对正常油门控制导致起飞/转换较慢的机型有帮助。当该值大于 THR_MIN 时使用，否则使用 THR_MIN。
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(takeoff_throttle_min,       "TKOFF_THR_MIN",    0),

    // @Param: TKOFF_THR_IDLE
    // @DisplayName: 起飞怠速油门
    // @Description: 解锁后、起飞前保持的怠速油门，适用于 TAKEOFF 与 AUTO 模式。
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(takeoff_throttle_idle,       "TKOFF_THR_IDLE",    0),

    // @Param: TKOFF_OPTIONS
    // @DisplayName: 起飞选项
    // @Description: 选择 AUTO 与 TAKEOFF 模式下的起飞方式。
    // @Bitmask: 0: 未置位时起飞始终使用最大油门（THR_MAX 或 TKOFF_THR_MAX）；置位后 TECS 可在最小（THR_MIN 或 TKOFF_THR_MIN）与最大（THR_MAX 或 TKOFF_THR_MAX）之间工作，仅在使用空速传感器时生效。
    // @User: Advanced
    ASCALAR(takeoff_options,               "TKOFF_OPTIONS",       0),
    
    // @Param: TKOFF_TDRAG_ELEV
    // @DisplayName: 起飞尾轮机升降舵
    // @Description: 设置起飞初期施加的升降舵量。用于将尾轮机尾轮压地，以获得最大地面转向。应与 TKOFF_TDRAG_SPD1、GROUND_STEER_ALT 及地面转向控制器调参配合。设为 0 表示跳过“压尾”阶段；手抛与弹射起飞应设为 0。尾轮机通常设为 100（起飞初期全上升舵）。大多数前三点起落架机型设为 0 即可；部分机型可设小负值（约 -20~-30）以压住前轮。仅在前轮抓地不足时使用负值，过大的下俯可能导致转向不稳，建议每次增加 10% 调整。
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(takeoff_tdrag_elevator,     "TKOFF_TDRAG_ELEV",  0),

    // @Param: TKOFF_TDRAG_SPD1
    // @DisplayName: 起飞尾轮机速度1
    // @Description: 设置停止压尾并转入方向舵地面转向控制的空速。当达到 TKOFF_TDRAG_SPD1 时，飞机俯仰保持水平直至达到 TKOFF_ROTATE_SPD，此时使用任务中指定的起飞俯仰“抬机头”爬升。设为 0 则直接进入抬机头阶段。手抛与弹射起飞应设为 0。前三点起落架机型也应设为 0，除非采用轻微压前轮的方式。尾轮机应设在略低于失速速度处。
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_tdrag_speed1,     "TKOFF_TDRAG_SPD1",  0),

    // @Param: TKOFF_ROTATE_SPD
    // @DisplayName: 起飞抬机头速度
    // @Description: 设定飞机开始“抬机头”并使用任务爬升俯仰的空速。若为 0，则起飞开始即使用爬升俯仰。手抛与弹射起飞应设为 0。地面起飞应设在失速速度以上，通常高出 10%-30%。滑跑期间可用 TKOFF_GND_PITCH 在该空速以下保持机体贴地。
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_rotate_speed,     "TKOFF_ROTATE_SPD",  0),

    // @Param: TKOFF_THR_SLEW
    // @DisplayName: 起飞油门斜率
    // @Description: 自动起飞期间的油门斜率。为 0 时使用 THR_SLEWRATE。滚行起飞可适当降低斜率以减缓加速并改善地面转向。该值为每秒油门百分比变化量，例如 20 表示 5 秒内推至目标油门。不建议低于 20，可能导致油门不足而提前爬升。-1 表示起飞不限制斜率。
    // @Units: %/s
    // @Range: -1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(takeoff_throttle_slewrate, "TKOFF_THR_SLEW",  0),

    // @Param: TKOFF_PLIM_SEC
    // @DisplayName: 起飞俯仰限幅衰减
    // @Description: 在接近目标高度前数秒降低自动起飞的最小俯仰限制，使飞控提前开始拉平以减少超调。设为 0 表示在到达并穿过目标高度前都强制任务俯仰最小值；否则在最后阶段逐步降为 0。该值作用于 pitch_min 而非俯仰需求，飞控仍会指令爬升，只是不再强制更高俯仰。
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
    GSCALAR(takeoff_pitch_limit_reduction_sec, "TKOFF_PLIM_SEC",  2),

    // @Param: TKOFF_FLAP_PCNT
    // @DisplayName: 起飞襟翼百分比
    // @Description: 自动起飞时施加的襟翼量（百分比）
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    GSCALAR(takeoff_flap_percent,     "TKOFF_FLAP_PCNT", 0),

    // @Param: LEVEL_ROLL_LIMIT
    // @DisplayName: 平飞横滚限制
    // @Description: 在需要平飞的模式（如降落末段、自动起飞）中限制最大倾斜角。该值应较小（如 5 度）以避免起降时机翼触地。设为 0 会在自动起飞 5 米以下及最终进近拉平段完全禁用航向保持。
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Standard
    GSCALAR(level_roll_limit,              "LEVEL_ROLL_LIMIT",   5),

    // @Param: USE_REV_THRUST
    // @DisplayName: 允许反推的阶段位掩码
    // @Description: 控制何时使用反推。设为非零时，每一位对应可使用反推的飞行阶段。常用值为 2（仅 AUTO_LAND），即仅在 AUTO 降落阶段启用反推；另一个常用值为 1（所有自动飞行阶段）。若 THR_MIN < 0，MANUAL 模式下始终可反推。非自动油门模式下若不使用反推，该模式的 THR_MIN 等效为 0。
    // @Bitmask: 0:AUTO_ALWAYS,1:AUTO_LAND,2:AUTO_LOITER_TO_ALT,3:AUTO_LOITER_ALL,4:AUTO_WAYPOINTS,5:LOITER,6:RTL,7:CIRCLE,8:CRUISE,9:FBWB,10:GUIDED,11:AUTO_LANDING_PATTERN,12:FBWA,13:ACRO,14:STABILIZE,15:THERMAL
    // @User: Advanced
    GSCALAR(use_reverse_thrust,     "USE_REV_THRUST",  float(UseReverseThrust::AUTO_LAND_APPROACH)),

    // @Param: ALT_OFFSET
    // @DisplayName: 高度偏移
    // @Description: 自动飞行中加到目标高度上的全局偏移，可用于给任务增加统一高度偏置
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Advanced
    GSCALAR(alt_offset, "ALT_OFFSET",                 0),

    // @Param: WP_RADIUS
    // @DisplayName: 航点半径
    // @Description: 设定判定航点完成的最大距离。为避免飞机偏离超过 WP_RADIUS 时绕圈，会额外检查是否穿越过“终止线”（过航点且垂直于上一航段）。若穿越则判定完成。导航控制器可能根据转弯角度与速度在距航点大于 WP_RADIUS 时才转弯。WP_RADIUS 可设得大于机体常规转弯半径，控制器会自行计算转弯时机；过小则易转弯过冲。
    // @Units: m
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_MAX_RADIUS
    // @DisplayName: 航点最大半径
    // @Description: 设定航点判定完成的最大距离，覆盖默认“穿越终止线”逻辑。正常 AUTO 行为应设为 0。仅在必须接近到指定半径时才建议设为非零，飞机会绕圈直至进入半径；若机体转弯半径大于该值，可能导致无限绕圈。
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_max_radius,        "WP_MAX_RADIUS",      0),

    // @Param: WP_LOITER_RAD
    // @DisplayName: 航点盘旋半径
    // @Description: 盘旋时保持的半径距离。设为负数表示默认盘旋方向为逆时针；设为接近 0 时，实际盘旋半径由 ROLL_LIMIT_DEG 决定。
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    ASCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

    // @Param: RTL_RADIUS
    // @DisplayName: RTL 盘旋半径
    // @Description: RTL 模式下的盘旋半径。为 0 则使用 WP_LOITER_RAD。负值为逆时针，正值为顺时针。对 Q_RTL_MODE=1 的 QuadPlane，该值用于设定固定翼转 VTOL 降落的最小半径。
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_radius,             "RTL_RADIUS",  0),
    
    // @Param: STALL_PREVENTION
    // @DisplayName: 启用失速保护
    // @Description: 在限滚模式的低空速下启用横滚限制。限制基于转弯气动载荷因子，并随 AIRSPEED_MIN 变化，需正确设置。无空速传感器时使用基于风速估计的合成空速，可能存在误差。
    // @Values: 0:禁用,1:启用
    // @User: Standard
    ASCALAR(stall_prevention, "STALL_PREVENTION",  1),

    // @Param: AIRSPEED_CRUISE
    // @DisplayName: 目标巡航空速
    // @Description: 自动油门模式下的目标巡航空速（m/s），为指示/校准空速。
    // @Units: m/s
    // @User: Standard
    ASCALAR(airspeed_cruise,     "AIRSPEED_CRUISE",  AIRSPEED_CRUISE),

    // @Param: AIRSPEED_MIN
    // @DisplayName: 最小空速
    // @Description: 自动油门模式下的最小空速需求，应设为平飞失速速度的 1.2 倍左右。
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_min, "AIRSPEED_MIN",  AIRSPEED_FBW_MIN),

    // @Param: AIRSPEED_MAX
    // @DisplayName: 最大空速
    // @Description: 自动油门模式下的最大空速需求，应略小于 THR_MAX 平飞速度，并至少比 AIRSPEED_MIN 高 50%，以保证 TECS 高度控制精度。
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_max, "AIRSPEED_MAX",  AIRSPEED_FBW_MAX),

    // @Param: AIRSPEED_STALL
    // @DisplayName: 失速空速
    // @Description: 启用失速保护时，用于计算转弯时的最小空速。也用于降落末段作为 TECS 可请求的最小空速，使 TECS_LAND_ARSPD 或 LAND_PF_ARSPD 可实现低于 AIRSPEED_MIN 的着陆空速。设为 0 则假定该值等于最小空速。通常设置略高于真实失速速度。
    // @Units: m/s
    // @Range: 5 75
    // @User: Standard
    ASCALAR(airspeed_stall, "AIRSPEED_STALL", 0),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: FBW 升降舵反向
    // @Description: 在 FBWB 与 CRUISE 模式下反转升降舵方向。设为 0 时拉杆抬升高度；设为 1 时拉杆降低高度。
    // @Values: 0:禁用,1:启用
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),

#if AP_TERRAIN_AVAILABLE
    // @Param: TERRAIN_FOLLOW
    // @DisplayName: 启用地形跟随
    // @Description: 在 CRUISE、FBWB、RTL 与备降点启用地形跟随。需同时设置 TERRAIN_ENABLE=1 并由地面站提供地形数据。启用后 CRUISE/FBWB 将保持相对地形高度；RTL 的返航高度也按地形高度计算；备降点高度同样以地形为基准。该选项不影响任务航点（每个航点可单独选择相对 Home 或相对地形）。使用地形跟随任务需要地面站在创建任务时将航点类型设为地形高度。
    // @Bitmask: 0:启用所有模式, 1:FBWB, 2:Cruise, 3:Auto, 4:RTL, 5:Avoid_ADSB, 6:Guided, 7:Loiter, 8:Circle, 9:QRTL, 10:QLand, 11:Qloiter, 12:AUTOLAND
    // @User: Standard
    GSCALAR(terrain_follow, "TERRAIN_FOLLOW",  0),

    // @Param: TERRAIN_LOOKAHD
    // @DisplayName: 地形前视距离
    // @Description: 控制地形跟随向前看的距离，以确保高于前方地形。设为 0 表示不前视，仅跟踪正下方地形。在 AUTO 模式下前视距离不会超过下一个航点。
    // @Range: 0 10000
    // @Units: m
    // @User: Standard
    GSCALAR(terrain_lookahead, "TERRAIN_LOOKAHD",  2000),
#endif

    // @Param: FBWB_CLIMB_RATE
    // @DisplayName: FBW-B 高度变化率
    // @Description: FBWB/CRUISE 模式中满升降舵偏转时目标高度变化率（m/s）。实际爬升率可能受空速与油门控制限制而低于该值。若设为默认 2.0，则满偏 10 秒目标高度变化 20 米。
    // @Range: 1 10
    // @Units: m/s
	// @Increment: 0.1
    // @User: Standard
    GSCALAR(flybywire_climb_rate, "FBWB_CLIMB_RATE",  2.0f),

    // @Param: THR_MIN
    // @DisplayName: 最小油门
    // @Description: 在非 MANUAL 模式且未设置 THR_PASS_STAB 时的最小油门百分比。硬件支持时允许负值用于反推。
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: 最大油门
    // @Description: 在非 MANUAL 模式且未设置 THR_PASS_STAB 时的最大油门百分比。
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),

    // @Param: TKOFF_THR_MAX
    // @DisplayName: 起飞最大油门
    // @Description: 自动起飞时允许的最大油门。设为 0 则起飞时也使用 THR_MAX。
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    ASCALAR(takeoff_throttle_max,   "TKOFF_THR_MAX",        0),

    // @Param: THR_SLEWRATE
    // @DisplayName: 油门斜率
    // @Description: 每秒油门百分比最大变化量。下限受每循环舵机 1 微秒增量限制，可用 SCHED_LOOP_RATE 约除以 10 估算最小可达值。
    // @Units: %/s
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_slewrate,      "THR_SLEWRATE",   100),

    // @Param: FLAP_SLEWRATE
    // @DisplayName: 襟翼斜率
    // @Description: 每秒襟翼输出的最大百分比变化。设为 25 表示 1 秒内不超过全行程的 25%。设为 0 表示不限制速度。
    // @Units: %/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flap_slewrate,          "FLAP_SLEWRATE",   75),

    // @Param: THR_SUPP_MAN
    // @DisplayName: 油门抑制手动直通
    // @Description: 自动模式中油门抑制时默认强制为 0。启用后抑制期间改为手动油门，适用于油机在等待起飞时手动保持怠速。
	// @Values: 0:禁用,1:启用
    // @User: Advanced
    GSCALAR(throttle_suppress_manual,"THR_SUPP_MAN",   0),

    // @Param: THR_PASS_STAB
    // @DisplayName: 自稳模式油门直通
    // @Description: 启用后在 STABILIZE/FBWA/ACRO 中油门直接来自遥控器，THR_MIN 与 THR_MAX 不再生效。适用于油机通过油门切断开关将油门压到正常最小值以下的场景。
	// @Values: 0:禁用,1:启用
    // @User: Advanced
    GSCALAR(throttle_passthru_stabilize,"THR_PASS_STAB",   0),

    // @Param: THR_FAILSAFE
    // @DisplayName: 油门与 RC 失效保护使能
    // @Description: 0 禁用失效保护；1 在 RC 输入丢失时启用失效保护（通过油门低于 THR_FS_VALUE、接收机脉冲/数据丢失或接收机 FS 标志检测，例如 SBUS）。触发后执行可配置的失效保护动作，并忽略 RC 输入。2 表示检测到 RC 失效时不使用 RC 输入，但不触发失效保护动作。
    // @Values: 0:禁用,1:启用,2:启用但不触发
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   int(ThrFailsafe::Enabled)),


    // @Param: THR_FS_VALUE
    // @DisplayName: 油门失效保护阈值
    // @Description: 油门输入通道的 PWM 阈值，低于该值触发油门失效保护。应明显低于正常最小油门。
    // @Range: 925 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   950),

    // @Param: TRIM_THROTTLE
    // @DisplayName: 巡航油门百分比
    // @Description: 自动油门模式下的目标油门百分比，用于维持 AIRSPEED_CRUISE。注意：电池末期电压下降可能需要更高油门才能保持空速。
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_cruise,        "TRIM_THROTTLE",  AP_PLANE_TRIM_THROTTLE_DEFAULT),

    // @Param: THROTTLE_NUDGE
    // @DisplayName: 油门微调使能
    // @Description: 启用后在自动油门模式下通过油门输入“微调”油门或空速。带空速传感器时，油门超过 50% 会将目标空速从 AIRSPEED_CRUISE 提高至 AIRSPEED_MAX；无空速传感器时，油门超过 50% 会提升目标油门。
    // @Values: 0:禁用,1:启用
    // @User: Standard
    GSCALAR(throttle_nudge,         "THROTTLE_NUDGE",  1),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: 短失效保护动作
    // @Description: 短失效保护触发时的动作。短失效可由 RC 丢失或油门阈值触发（见 THR_FS_VALUE）。在 CIRCLE 或 RTL 模式下该参数忽略。自稳/手动模式下，FS_SHORT_ACTN=0/1 切到 CIRCLE；=2 切到 FBWA 且油门为 0；=4 切到 FBWB。其他模式（AUTO/GUIDED/LOITER）下，FS_SHORT_ACTN=0 不切模式；=1 切到 CIRCLE；=2 切到 FBWA 且油门为 0；=4 切到 FBWB。FS_LONG_TIMEOUT 之后的行为见 FS_LONG_ACTN 文档。该参数仅适用于固定翼模式；QuadPlane 模式会切到 QLAND，除非设置 Q_OPTIONS 位 5（QRTL）或 20（RTL）。
    // @Values: 0:CIRCLE/不变(已在 AUTO|GUIDED|LOITER),1:CIRCLE,2:FBWA 且油门为 0,3:禁用,4:FBWB
    // @User: Standard
    GSCALAR(fs_action_short,        "FS_SHORT_ACTN",  FS_ACTION_SHORT_BESTGUESS),

    // @Param: FS_LONG_ACTN
    // @DisplayName: 长失效保护动作
    // @Description: 长失效（持续 FS_LONG_TIMEOUT 秒）触发时的动作。若失效开始时在自稳/手动模式，则 FS_LONG_ACTN=0/1 切到 RTL，=2 切到 FBWA。若失效开始时在自动模式（AUTO/GUIDED），则 FS_LONG_ACTN=0 保持原模式，=1 切到 RTL，=2 切到 FBWA。=3 部署降落伞（确保已配置并启用）；=4 切到 AUTO 并使用当前航点（若未处于降落序列）；=5 尽可能切到 AUTOLAND，否则 RTL。该参数仅适用于固定翼模式；QuadPlane 模式会切到 QLAND，除非设置 Q_OPTIONS 位 5（QRTL）或 20（RTL）。
    // @Values: 0:继续,1:返航,2:滑翔,3:释放降落伞,4:自动,5:AUTOLAND
    // @User: Standard
    GSCALAR(fs_action_long,         "FS_LONG_ACTN",   FS_ACTION_LONG_CONTINUE),

    // @Param: FS_LONG_TIMEOUT
    // @DisplayName: 长失效保护超时
    // @Description: 失效条件持续达到该时间（秒）后触发长失效保护事件，默认 5 秒。
    // @Units: s
    // @Range: 1 300
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout_long,        "FS_LONG_TIMEOUT", 5),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS 失效保护使能
    // @Description: 启用地面站遥测失效保护。若在 FS_LONG_TIMEOUT 秒内未收到 MAVLink HEARTBEAT，则触发失效保护。FS_GCS_ENABL=1：未收到 HEARTBEAT 即触发（仅在收到主地面站 MAV_GCS_SYSID 的首个心跳后开始计时）。=2：HEARTBEAT 丢失或 RADIO_STATUS.remrssi 为 0 时触发（地面站收不到飞机状态，可能因地面站/机载电台噪声不对称导致单向链路）。=3：仅在 AUTO 模式下按心跳触发。警告：启用该选项可能导致在地面失联时进入失效保护并启动电机。若为电动机型，建议同时启用 ARMING_REQUIRED。
    // @Values: 0:禁用,1:心跳,2:心跳+REMRSSI,3:心跳+AUTO
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_FAILSAFE_OFF),

    // @Param: FLTMODE_CH
    // @DisplayName: 飞行模式通道
    // @Description: 用于飞行模式控制的 RC 通道
    // @Values: 0:禁用,1:通道1,2:通道2,3:通道3,4:通道4,5:通道5,6:通道6,7:通道7,8:通道8,9:通道9,10:通道10,11:通道11,12:通道12,13:通道13,14:通道14,15:通道15,16:通道16
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: 飞行模式1
    // @Description: 开关位置 1 的飞行模式（910-1230 与 2049 以上）
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,13:TAKEOFF,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE,23:QACRO,24:THERMAL,25:Loiter to QLand,26:AUTOLAND
    // @User: Standard
    GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: 飞行模式2
    // @Description: 开关位置 2 的飞行模式（1231-1360）
    GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: 飞行模式3
    // @Description: 开关位置 3 的飞行模式（1361-1490）
    GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: 飞行模式4
    // @Description: 开关位置 4 的飞行模式（1491-1620）
    GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: 飞行模式5
    // @Description: 开关位置 5 的飞行模式（1621-1749）
    GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: 飞行模式6
    // @Description: 开关位置 6 的飞行模式（1750-2049）
    GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

    // @Param: INITIAL_MODE
    // @DisplayName: 初始飞行模式
    // @Description: 启动时进入的模式，适合无接收机时开机直接进入 AUTO。
    // @CopyValuesFrom: FLTMODE1
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     Mode::Number::MANUAL),

    // @Param: ROLL_LIMIT_DEG
    // @DisplayName: 最大倾斜角
    // @Description: 在稳定限幅模式中允许的最大倾斜角。增大可更急转弯，减小可防止加速失速。
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    ASCALAR(roll_limit,          "ROLL_LIMIT_DEG",    ROLL_LIMIT_DEG),

    // @Param: PTCH_LIM_MAX_DEG
    // @DisplayName: 最大俯仰角
    // @Description: 在稳定限幅模式中允许的最大上俯仰角。
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_max,     "PTCH_LIM_MAX_DEG",  PITCH_MAX),

    // @Param: PTCH_LIM_MIN_DEG
    // @DisplayName: 最小俯仰角
    // @Description: 在稳定限幅模式中允许的最大下俯仰角。
    // @Units: deg
    // @Range: -90 0
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_min,     "PTCH_LIM_MIN_DEG",  PITCH_MIN),

    // @Param: ACRO_ROLL_RATE
    // @DisplayName: ACRO 横滚角速度
    // @Description: ACRO 模式下摇杆打满时的最大横滚角速度
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_roll_rate,          "ACRO_ROLL_RATE",    180),

    // @Param: ACRO_PITCH_RATE
    // @DisplayName: ACRO 俯仰角速度
    // @Description: ACRO 模式下摇杆打满时的最大俯仰角速度
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_pitch_rate,          "ACRO_PITCH_RATE",  180),

    // @Param: ACRO_YAW_RATE
    // @DisplayName: ACRO 偏航角速度
    // @Description: ACRO 模式下摇杆打满时的最大偏航角速度。若设为 0，则方向舵直接由摇杆控制。仅在 YAW_RATE_ENABLE=1 时可用。
    // @Units: deg/s
    // @Range: 0 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_yaw_rate,            "ACRO_YAW_RATE",    0),
    
    // @Param: ACRO_LOCKING
    // @DisplayName: ACRO 姿态锁定
    // @Description: 松杆时启用姿态锁定。设为 2 且启用偏航角速度控制器时使用四元数锁定，可保持任意姿态。
    // @Values: 0:禁用,1:启用,2:四元数
    // @User: Standard
    GSCALAR(acro_locking,             "ACRO_LOCKING",     0),

    // @Param: GROUND_STEER_ALT
    // @DisplayName: 地面转向高度
    // @Description: 使用方向舵地面转向控制器的高度阈值。非零时，在距 Home 高度该范围内使用 STEER2SRV 控制方向舵。
    // @Units: m
    // @Range: -100 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(ground_steer_alt,         "GROUND_STEER_ALT",   0),

    // @Param: GROUND_STEER_DPS
    // @DisplayName: 地面转向速率
    // @Description: 方向舵打满时的地面转向角速度（度/秒）
    // @Units: deg/s
    // @Range: 10 360
    // @Increment: 1
    // @User: Advanced
    GSCALAR(ground_steer_dps,         "GROUND_STEER_DPS",  90),

    // @Param: MIXING_GAIN
    // @DisplayName: 混控增益
    // @Description: V 尾与副翼/升降舵混控输出增益。默认 0.5 可避免饱和并保留控制余量；硬件混控常为 1.0，行程更大但易饱和。若启用 VTAIL_OUTPUT 或 ELEVON_OUTPUT 后舵面行程不足，可提高 MIXING_GAIN。混控输出范围 900~2100 微秒。
    // @Range: 0.5 1.2
    // @User: Standard
    GSCALAR(mixing_gain,            "MIXING_GAIN",    0.5f),

    // @Param: RUDDER_ONLY
    // @DisplayName: 仅方向舵机型
    // @Description: 启用仅方向舵模式。方向舵将在姿态控制模式（如 FBWA）中控制姿态。请将横滚杆输入映射到 RCMAP_YAW（通常通道 4），并将方向舵舵机接在该通道。启用后自动地面转向将被禁用。建议将 KFF_RDDRMIX 设为 1.0，并根据机型调整 YAW2SRV_DAMP（建议起始值 0.5）。
    // @Values: 0:禁用,1:启用
    // @User: Standard
    GSCALAR(rudder_only,             "RUDDER_ONLY",  0),

    // @Param: MIXING_OFFSET
    // @DisplayName: 混控偏置
    // @Description: V 尾与副翼/升降舵混控输出的偏置百分比。可与 MIXING_GAIN 配合调整舵面对输入的响应。设为正/负值可增强副翼或升降舵的响应。常见用法是为飞翼的副翼/升降舵设置正值以增强副翼响应。默认 0 表示副翼与升降舵输入响应相同。
    // @Units: d%
    // @Range: -1000 1000
    // @User: Standard
    GSCALAR(mixing_offset,          "MIXING_OFFSET",  0),

    // @Param: DSPOILR_RUD_RATE
    // @DisplayName: 差动扰流板方向舵比例
    // @Description: 方向舵输出作用到差动扰流板的比例（百分比）。默认 100 表示方向舵打满时扰流板全偏；设为 0 则差动扰流板完全跟随副翼/升降舵（无方向舵作用）。
    // @Units: %
    // @Range: -100 100
    // @User: Standard
    GSCALAR(dspoiler_rud_rate,      "DSPOILR_RUD_RATE",  DSPOILR_RUD_RATE_DEFAULT),

    // @Param: LOG_BITMASK
    // @DisplayName: 日志位掩码
    // @Description: 启用机载日志类型的位掩码，为各日志类型之和。通常建议设为 65535 以开启全部基础日志。
    // @Bitmask: 0:快速姿态,1:中速姿态,2:GPS,3:性能,4:控制调参,5:导航调参,7:IMU,8:任务指令,9:电池监测,10:指南针,11:TECS,12:相机,13:RC 输入输出,14:测距仪,19:原始 IMU,20:全速姿态,21:视频稳定,22:全速陷波
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),

    // @Param: SCALING_SPEED
    // @DisplayName: 速度缩放计算用速度
    // @Description: 计算舵面速度缩放所用的空速（m/s）。修改该值会影响所有 PID。
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GROUNDSPEED
    // @DisplayName: 最小地速
    // @Description: 空速控制下允许的最小地速
    // @Units: m/s
    // @User: Advanced
    ASCALAR(min_groundspeed,      "MIN_GROUNDSPEED",  MIN_GROUNDSPEED),

    // @Param: PTCH_TRIM_DEG
    // @DisplayName: 俯仰角偏移
    // @Description: 用于平飞空中配平的俯仰角偏移（度）。也可通过正确的地面水平校准来替代该参数。
    // @Units: deg
    // @Range: -45 45
    // @User: Standard
    GSCALAR(pitch_trim,             "PTCH_TRIM_DEG",  0.0f),

    // @Param: RTL_ALTITUDE
    // @DisplayName: RTL 高度
    // @Description: RTL 模式下相对 Home 的目标高度。设为 -1 则保持当前高度。若不返回 Home，则使用备降点高度。
    // @Units: m
    // @User: Standard
    GSCALAR(RTL_altitude,        "RTL_ALTITUDE",   ALT_HOLD_HOME),

    // @Param: CRUISE_ALT_FLOOR
    // @DisplayName: FBWB/CRUISE 最小高度
    // @Description: FBWB 与 CRUISE 模式允许的最小高度（相对 Home，米）。低于该高度将拉平，并在这些模式下强制爬升至该高度。设为 0 表示不限制。
    // @Units: m
    // @User: Standard
    GSCALAR(cruise_alt_floor,   "CRUISE_ALT_FLOOR", CRUISE_ALT_FLOOR),

    // @Param: FLAP_1_PERCNT
    // @DisplayName: 襟翼 1 百分比
    // @Description: 达到 FLAP_1_SPEED 时襟翼位置变化百分比。设为 0 可禁用襟翼。
    // @Range: 0 100
    // @Increment: 1
    // @Units: %
    // @User: Advanced
    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),

    // @Param: FLAP_1_SPEED
    // @DisplayName: 襟翼 1 速度
    // @Description: 达到该空速（m/s）时应用 FLAP_1_PERCENT 襟翼。注意 FLAP_1_SPEED 应大于或等于 FLAP_2_SPEED。
    // @Range: 0 100
	// @Increment: 1
    // @Units: m/s
    // @User: Advanced
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),

    // @Param: FLAP_2_PERCNT
    // @DisplayName: 襟翼 2 百分比
    // @Description: 达到 FLAP_2_SPEED 时襟翼位置变化百分比。设为 0 可禁用襟翼。
    // @Range: 0 100
	// @Units: %
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),

    // @Param: FLAP_2_SPEED
    // @DisplayName: 襟翼 2 速度
    // @Description: 达到该空速（m/s）时应用 FLAP_2_PERCENT 襟翼。注意 FLAP_1_SPEED 应大于或等于 FLAP_2_SPEED。
    // @Range: 0 100
	// @Units: m/s
	// @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),

#if HAL_WITH_IO_MCU
    // @Param: OVERRIDE_CHAN
    // @DisplayName: IO 覆盖通道
    // @Description: 设为非零时，用该 RC 输入通道在带 IO 协处理器的板上实现 FMU 主控失效时的 IO 手动控制。该通道超过 1750 后，FMU 不再控制舵机，由 IO 直接控制。FMU 崩溃时 IO 手动控制会自动启用。该参数便于在不让 FMU 崩溃的情况下测试手动行为，也可用于外部覆盖控制板。可将 OVERRIDE_CHAN 与 FLTMODE_CH 设为同一通道，在飞行模式 6 时触发 IO 覆盖。注意：因 FMU 崩溃触发覆盖时，FMU 的 6 路辅输出不再更新，因此需要将飞控通道分配在 IOMCU 的前 8 路输出上。
    // @Range: 0 16
    // @Increment: 1
    // @User: Advanced
    GSCALAR(override_channel,      "OVERRIDE_CHAN",  0),
#endif

    // @Param: RTL_AUTOLAND
    // @DisplayName: RTL 自动降落
    // @Description: 到达 RTL 位置后自动进入降落序列。需添加 DO_LAND_START 任务项作为降落序列起点标记，并会选择离当前位置最近的降落序列。值为 1 时，若备降点在范围内则用备降点替代 HOME（详见备降点文档）。若设为 0 且任务中包含 DO_LAND_START 或 DO_RETURN_PATH_START，则会触发解锁检查失败。可设为 3 以避免解锁检查失败，并将 DO_LAND_START 仅用于复飞（详见自动降落中止说明），且不改变 RTL 行为。
    // @Values: 0:禁用,1:先返航再按 DO_LAND_START 降落,2:直接进入 DO_LAND_START 降落序列,3:仅用于复飞,4:直接进入 DO_RETURN_PATH_START 降落序列
    // @User: Standard
    GSCALAR(rtl_autoland,         "RTL_AUTOLAND",   float(RtlAutoland::RTL_DISABLE)),

    // @Param: CRASH_ACC_THRESH
    // @DisplayName: 碰撞减速度阈值
    // @Description: X 轴减速度阈值，用于通知碰撞检测器可能发生撞击，以便坠机后快速上锁电机。该值应显著高于正常飞行的 X 轴负加速度，可通过日志查看 IMU.x 平均值。值越大灵敏度越低（需更大冲击才触发）。电机震动较小的电动机型建议 25（约 2.5G），油机/硝基机型需更高。设为 0 禁用碰撞检测。
    // @Units: m/s/s
    // @Range: 10 127
    // @Increment: 1
    // @User: Advanced
    GSCALAR(crash_accel_threshold,          "CRASH_ACC_THRESH",   0),

    // @Param: CRASH_DETECT
    // @DisplayName: 碰撞检测
    // @Description: 在 AUTO 飞行中自动检测坠机并执行所选动作。上锁会关闭电机以确保安全并避免烧毁电调/电机。设为 0 禁用。
    // @Bitmask: 0:上锁
    // @User: Advanced
    ASCALAR(crash_detection_enable,         "CRASH_DETECT",   0),

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS 驱动
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

#if AP_CAMERA_ENABLED
    // @Group: CAM
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera, "CAM", AP_Camera),
#endif

    // @Group: ARMING_
    // @Path: AP_Arming_Plane.cpp,../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Plane),

#if AP_RELAY_ENABLED
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY", AP_Relay),
#endif

#if HAL_PARACHUTE_ENABLED
	// @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute,		"CHUTE_", AP_Parachute),
#endif

#if AP_RANGEFINDER_ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,            "RNGFND", RangeFinder),

    // @Param: RNGFND_LANDING
    // @DisplayName: 启用测距仪
    // @Description: 设置测距仪用于自动降落等场景。启用起降时，用于降落进近与最终拉平、VTOL 降落、起飞以及近地油门抑制；启用辅助时用于 VTOL 辅助；启用爬升时用于 QRTL 与 AUTO 的初始爬升。设为 0 禁用测距仪。
    // @Bitmask: 0:全部, 1:起飞与降落, 2:辅助, 3:初始爬升
    // @User: Standard
    GSCALAR(rangefinder_landing,    "RNGFND_LANDING",   0),
#endif

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

#if HAL_ADSB_ENABLED
    // @Group: ADSB_
    // @Path: ../libraries/AP_ADSB/AP_ADSB.cpp
    GOBJECT(adsb,                "ADSB_", AP_ADSB),
#endif  // HAL_ADSB_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
    // @Group: AVD_
    // @Path: ../libraries/AP_Avoidance/AP_Avoidance.cpp
    GOBJECT(avoidance_adsb, "AVD_", AP_Avoidance_Plane),
#endif  // AP_ADSB_AVOIDANCE_ENABLED

#if HAL_QUADPLANE_ENABLED
    // @Group: Q_
    // @Path: quadplane.cpp
    GOBJECT(quadplane,           "Q_", QuadPlane),
#endif

#if AP_TUNING_ENABLED
    // @Group: TUNE_
    // @Path: tuning.cpp,../libraries/AP_Tuning/AP_Tuning.cpp
    GOBJECT(tuning,           "TUNE_", AP_Tuning_Plane),
#endif

#if HAL_QUADPLANE_ENABLED
    // @Group: Q_A_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    { "Q_A_", (const void *)&plane.quadplane.attitude_control,
      {group_info : AC_AttitudeControl_Multi::var_info}, AP_PARAM_FLAG_POINTER,
      Parameters::k_param_q_attitude_control, AP_PARAM_GROUP },
#endif

    // @Group: RLL
    // @Path: ../libraries/APM_Control/AP_RollController.cpp
    GOBJECT(rollController,         "RLL",   AP_RollController),

    // @Group: PTCH
    // @Path: ../libraries/APM_Control/AP_PitchController.cpp
    GOBJECT(pitchController,        "PTCH",  AP_PitchController),

    // @Group: YAW
    // @Path: ../libraries/APM_Control/AP_YawController.cpp
    GOBJECT(yawController,          "YAW",   AP_YawController),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),

	// 不在 g 类中、但包含 EEPROM 保存变量的对象

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                "RCMAP_",         RCMapper),

    // SR0 through SR6 were here

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // Airspeed 曾在此

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: TECS_
    // @Path: ../libraries/AP_TECS/AP_TECS.cpp
    GOBJECT(TECS_controller,         "TECS_",   AP_TECS),

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT", AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

#if AP_ADVANCEDFAILSAFE_ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    GOBJECT(afs,  "AFS_", AP_AdvancedFailsafe),
#endif

#if AP_OPTICALFLOW_ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", AP_OpticalFlow),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

#if HAL_RALLY_ENABLED
    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,  "RALLY_",       AP_Rally),
#endif

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group: 
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),
    
    // @Group: LAND_
    // @Path: ../libraries/AP_Landing/AP_Landing.cpp
    GOBJECT(landing, "LAND_", AP_Landing),

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

    // @Group: TKOFF_
    // @Path: mode_takeoff.cpp
    GOBJECT(mode_takeoff, "TKOFF_", ModeTakeoff),

#if MODE_AUTOLAND_ENABLED
    // @Group: AUTOLAND_
    // @Path: mode_autoland.cpp
    GOBJECT(mode_autoland, "AUTOLAND_", ModeAutoLand),
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    // @Group: PUP_
    // @Path: pullup.cpp
    GOBJECTN(mode_auto.pullup, pullup, "PUP_", GliderPullup),
#endif
    
    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

#if AP_QUICKTUNE_ENABLED
    // @Group: QWIK_
    // @Path: ../libraries/AP_Quicktune/AP_Quicktune.cpp
    GOBJECT(quicktune, "QWIK_",  AP_Quicktune),
#endif

#if HAL_GCS_ENABLED
    // @Group: MAV
    // @Path: ../libraries/GCS_MAVLink/GCS.cpp
    GOBJECT(_gcs,           "MAV",  GCS),
#endif

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

#if HAL_BUTTON_ENABLED
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    AP_SUBGROUPPTR(button_ptr, "BTN_", 1, ParametersG2, AP_Button),
#endif

#if AP_ICENGINE_ENABLED
    // @Group: ICE_
    // @Path: ../libraries/AP_ICEngine/AP_ICEngine.cpp
    AP_SUBGROUPINFO(ice_control, "ICE_", 2, ParametersG2, AP_ICEngine),
#endif

    // 3 曾用于 servo_channels 的原型

    // 4 曾用于 SYSID_ENFORCE

    // AP_Stats 曾为 5

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 6, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 7, ParametersG2, RC_Channels_Plane),
    
#if HAL_SOARING_ENABLED
    // @Group: SOAR_
    // @Path: ../libraries/AP_Soaring/AP_Soaring.cpp
    AP_SUBGROUPINFO(soaring_controller, "SOAR_", 8, ParametersG2, SoaringController),
#endif
  
    // @Param: RUDD_DT_GAIN
    // @DisplayName: 方向舵差动推力增益
    // @Description: 方向舵到差动推力的增益
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RUDD_DT_GAIN", 9, ParametersG2, rudd_dt_gain, 10),

    // @Param: MANUAL_RCMASK
    // @DisplayName: 手动 RC 直通掩码
    // @Description: MANUAL 模式下将 RC 通道直接直通到对应输出通道的掩码。非 MANUAL 模式下这些通道按正常逻辑工作。该参数用于在 MANUAL 下配合发射机混控实现复杂混控策略。启用后需认真进行飞前检查，确保 MANUAL 与非 MANUAL 模式下输出均正确。
    // @Bitmask: 0:通道1,1:通道2,2:通道3,3:通道4,4:通道5,5:通道6,6:通道7,7:通道8,8:通道9,9:通道10,10:通道11,11:通道12,12:通道13,13:通道14,14:通道15,15:通道16
    // @User: Advanced
    AP_GROUPINFO("MANUAL_RCMASK", 10, ParametersG2, manual_rc_mask, 0),
    
    // @Param: HOME_RESET_ALT
    // @DisplayName: Home 复位高度阈值
    // @Description: 未解锁时若飞机在 Home 航点的该高度范围内，将自动更新 Home 位置。设为 0 表示持续重置。
    // @Values: -1:从不重置,0:始终重置
    // @Range: -1 127
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("HOME_RESET_ALT", 11, ParametersG2, home_reset_threshold, 0),

    // 12 曾为 AP_Gripper

    // @Param: FLIGHT_OPTIONS
    // @DisplayName: 飞行模式选项
    // @Description: 飞行模式相关选项
    // @Bitmask: 0: 仅在直飞模式中启用方向舵混控（Manual/Stabilize/Acro）
    // @Bitmask: 1: 在 Cruise/FBWB 中使用中位油门表示配平空速
    // @Bitmask: 2: 起飞解锁时禁用姿态检查
    // @Bitmask: 3: 在 Cruise/FBWB 中强制目标空速为配平空速
    // @Bitmask: 4: RTL 转弯前先爬升到 RTL_ALTITUDE
    // @Bitmask: 5: 在 ACRO 模式启用偏航阻尼
    // @Bitmask: 6: 自动起飞期间抑制速度缩放使其不超过 1，以防无空速传感器振荡
    // @Bitmask: 7: 起飞启用默认空速
    // @Bitmask: 8: GCS 姿态仪不显示 PTCH_TRIM_DEG
    // @Bitmask: 9: OSD 姿态仪不显示 PTCH_TRIM_DEG
    // @Bitmask: 10: 非自动油门模式（除 MANUAL）将中位油门调整为 TRIM_THROTTLE
    // @Bitmask: 11: 地面模式下不抑制固定翼角速度增益
    // @Bitmask: 12: 启用 FBWB 风格盘旋高度控制
    // @Bitmask: 13: 起飞等待方向舵中位时用舵面提示
    // @Bitmask: 14: AUTO 中直接爬升到下一航点高度（非线性爬升）
    // @Bitmask: 15: 手动模式启用自动襟翼，襟翼设定使用目标与实际速度的最小值
    // @Bitmask: 16: 空速传感器启用且 AIRSPEED_STALL 设置时，启用完整气动载荷因子滚转限幅
    // @User: Advanced
    AP_GROUPINFO("FLIGHT_OPTIONS", 13, ParametersG2, flight_options, 0),

    // 14 was AP_Scripting

    // @Param: TKOFF_ACCEL_CNT
    // @DisplayName: 起飞加速度计数
    // @Description: 使用 TKOFF_THR_MINACC 解锁所需的加速度事件次数。默认 1 表示一次超过阈值的前向加速度即可解锁；设为大于 1 可要求多次前后运动。
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("TKOFF_ACCEL_CNT", 15, ParametersG2, takeoff_throttle_accel_count, 1),

#if AP_LANDINGGEAR_ENABLED
    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    AP_SUBGROUPINFO(landing_gear, "LGR_", 16, ParametersG2, AP_LandingGear),
#endif

    // @Param: DSPOILER_CROW_W1
    // @DisplayName: 差动扰流板鸭式襟翼外侧权重
    // @Description: 用于外侧两片舵面的差动扰流板/鸭式襟翼偏转量（0-100）。0 表示不使用鸭式襟翼，建议起始值 25。
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_CROW_W1", 17, ParametersG2, crow_flap_weight_outer, 0),

    // @Param: DSPOILER_CROW_W2
    // @DisplayName: 差动扰流板鸭式襟翼内侧权重
    // @Description: 用于内侧两片舵面的差动扰流板/鸭式襟翼偏转量（0-100）。0 表示不使用鸭式襟翼，建议起始值 45。
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_CROW_W2", 18, ParametersG2, crow_flap_weight_inner, 0),

    // @Param: TKOFF_TIMEOUT
    // @DisplayName: 起飞超时
    // @Description: 自动起飞超时。若非零且在该时间内地速未达到至少 4 m/s，则中止起飞并上锁。为 0 表示不限制超时。
    // @Range: 0 120
    // @Increment: 1
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("TKOFF_TIMEOUT", 19, ParametersG2, takeoff_timeout, 0),

    // @Param: DSPOILER_OPTS
    // @DisplayName: 差动扰流板与鸭式襟翼选项
    // @Description: 差动扰流板与鸭式襟翼选项。渐进式鸭式襟翼：先仅襟翼（0-50%），再进入鸭式襟翼（50-100%）。
    // @Bitmask: 0:俯仰输入, 1:每侧两舵面同时用于滚转, 2:渐进式鸭式襟翼
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_OPTS", 20, ParametersG2, crow_flap_options, 3),

    // @Param: DSPOILER_AILMTCH
    // @DisplayName: 差动扰流板副翼匹配
    // @Description: 缩放内侧襟翼的下偏幅度，使差动扰流板/全翼副翼不必用到全下偏行程。100 表示使用全行程，上偏不受影响。
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_AILMTCH", 21, ParametersG2, crow_flap_aileron_matching, 100),


    // 22 曾为 EFI

    // @Param: FWD_BAT_VOLT_MAX
    // @DisplayName: 前向油门电压补偿最大电压
    // @Description: 前向油门电压补偿的最大电压（高于该电压不再增加推力缩放）。建议 4.2×电芯数，0 表示禁用。建议 THR_MAX 不高于 100×FWD_BAT_VOLT_MIN/FWD_BAT_VOLT_MAX，THR_MIN 不低于 -100×FWD_BAT_VOLT_MIN/FWD_BAT_VOLT_MAX，并相应设置爬升/下降率限制。
    // @Range: 6 35
    // @Units: V
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FWD_BAT_VOLT_MAX", 23, ParametersG2, fwd_batt_cmp.batt_voltage_max, 0.0f),

    // @Param: FWD_BAT_VOLT_MIN
    // @DisplayName: 前向油门电压补偿最小电压
    // @Description: 前向油门电压补偿的最小电压（低于该电压不再增加推力缩放）。建议 3.5×电芯数，0 表示禁用。建议 THR_MAX 不高于 100×FWD_BAT_VOLT_MIN/FWD_BAT_VOLT_MAX，THR_MIN 不低于 -100×FWD_BAT_VOLT_MIN/FWD_BAT_VOLT_MAX，并相应设置爬升/下降率限制。
    // @Range: 6 35
    // @Units: V
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FWD_BAT_VOLT_MIN", 24, ParametersG2, fwd_batt_cmp.batt_voltage_min, 0.0f),

    // @Param: FWD_BAT_IDX
    // @DisplayName: 前向油门电池补偿索引
    // @Description: 用于前向油门补偿的电池监测器索引
    // @Values: 0:第一电池, 1:第二电池
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("FWD_BAT_IDX", 25, ParametersG2, fwd_batt_cmp.batt_idx, 0),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF 失效保护方差阈值
    // @Description: 设置 VTOL 模式下用于检查导航健康的最大可接受罗盘与速度方差
    // @Values: 0.6:严格, 0.8:默认, 1.0:宽松
    // @Range: 0.6 1.0
    // @User: Advanced
    AP_GROUPINFO("FS_EKF_THRESH", 26, ParametersG2, fs_ekf_thresh, FS_EKF_THRESHOLD_DEFAULT),

    // @Param: RTL_CLIMB_MIN
    // @DisplayName: RTL 最小爬升
    // @Description: RTL 初始爬升阶段的最小爬升高度（m）。期间横滚角限制为 LEVEL_ROLL_LIMIT。
    // @Units: m
    // @Range: 0 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_CLIMB_MIN", 27, ParametersG2, rtl_climb_min, 0),

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // @Group: GUIDED_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(guidedHeading, "GUIDED_", 28, ParametersG2, AC_PID),
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

    // @Param: MAN_EXPO_ROLL
    // @DisplayName: 手动横滚 Expo
    // @Description: MANUAL/ACRO/TRAINING 模式下横滚输入的指数比例
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAN_EXPO_ROLL", 29, ParametersG2, man_expo_roll, 0),

    // @Param: MAN_EXPO_PITCH
    // @DisplayName: 手动俯仰 Expo
    // @Description: MANUAL/ACRO/TRAINING 模式下俯仰输入的指数比例
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAN_EXPO_PITCH", 30, ParametersG2, man_expo_pitch, 0),

    // @Param: MAN_EXPO_RUDDER
    // @DisplayName: 手动方向舵 Expo
    // @Description: MANUAL/ACRO/TRAINING 模式下方向舵输入的指数比例
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAN_EXPO_RUDDER", 31, ParametersG2, man_expo_rudder, 0),

    // @Param: ONESHOT_MASK
    // @DisplayName: Oneshot 输出掩码
    // @Description: 使用 Oneshot 的输出通道掩码
    // @User: Advanced
    // @Bitmask: 0:舵机1, 1:舵机2, 2:舵机3, 3:舵机4, 4:舵机5, 5:舵机6, 6:舵机7, 7:舵机8, 8:舵机9, 9:舵机10, 10:舵机11, 11:舵机12, 12:舵机13, 13:舵机14, 14:舵机15, 15:舵机16, 16:舵机17, 17:舵机18, 18:舵机19, 19:舵机20, 20:舵机21, 21:舵机22, 22:舵机23, 23:舵机24, 24:舵机25, 25:舵机26, 26:舵机27, 27:舵机28, 28:舵机29, 29:舵机30, 30:舵机31, 31:舵机32
     AP_GROUPINFO("ONESHOT_MASK", 32, ParametersG2, oneshot_mask, 0),

#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 33, ParametersG2, AP_Follow),
#endif

    // @Param: AUTOTUNE_AXES
    // @DisplayName: 自动调参轴掩码
    // @Description: 1 字节的自动调参轴位图
    // @Bitmask: 0:横滚,1:俯仰,2:偏航
    // @User: Standard
    AP_GROUPINFO("AUTOTUNE_AXES", 34, ParametersG2, axis_bitmask, 7),

#if AC_PRECLAND_ENABLED
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    AP_SUBGROUPINFO(precland, "PLND_", 35, ParametersG2, AC_PrecLand),
#endif

#if AP_RANGEFINDER_ENABLED
    // @Param: RNGFND_LND_ORNT
    // @DisplayName: 测距仪降落朝向
    // @Description: 降落检测所用测距仪朝向。正常下视测距仪应设为 Down，QuadPlane 尾座式的后视测距仪设为 Back。可用 Custom1/Custom2 自定义朝向，需与至少一个可用测距仪朝向匹配。
    // @Values: 4:后向, 25:向下, 101:自定义1, 102:自定义2
    // @User: Standard
    AP_GROUPINFO("RNGFND_LND_ORNT", 36, ParametersG2, rangefinder_land_orient, ROTATION_PITCH_270),
#endif

    // @Param: FWD_BAT_THR_CUT
    // @DisplayName: 前向油门电压切断阈值
    // @Description: 自动油门模式下，电池静置电压低于该值时切断油门（使用 FWD_BAT_IDX 对应电池）。设为 0 表示不因低电压切断油门，电机会持续运行直至电池耗尽。应设置为电机最低工作电压或仅产生最小推力的电压，以保留电池供电给电子设备与执行机构。
    // @Range: 0 35
    // @Units: V
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FWD_BAT_THR_CUT", 37, ParametersG2, fwd_batt_cmp.batt_voltage_throttle_cutoff, 0.0f),

#if AP_PLANE_SYSTEMID_ENABLED
    // @Group: SID
    // @Path: systemid.cpp
    AP_SUBGROUPINFO(systemid, "SID", 38, ParametersG2, AP_SystemID),
#endif
    
    // @Param: CLIMB_SLOPE_HGT
    // @DisplayName: 爬升坡度最小高度
    // @Description: 设置在航点间应用爬升坡度的最小相对 Home 高度。低于该高度时飞机会立即爬升，达到该高度后才恢复按轨迹飞行，以避免靠近障碍物时缓慢爬升的不安全行为。默认值适合大多数环境，可根据地形或任务需要调整。
    // @Units: m
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CLIMB_SLOPE_HGT", 39, ParametersG2, waypoint_climb_slope_height_min, 25),

    // @Param: GUIDED_TIMEOUT
    // @DisplayName: 外部引导命令超时
    // @Description: 超时内未收到外部引导命令则回退为普通 GUIDED 模式。
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("GUIDED_TIMEOUT", 40, ParametersG2, guided_timeout, 3.0f),

    AP_GROUPEND
};

ParametersG2::ParametersG2(void) :
    unused_integer{1}
#if HAL_BUTTON_ENABLED
    ,button_ptr(&plane.button)
#endif
#if HAL_SOARING_ENABLED
    ,soaring_controller(plane.TECS_controller, plane.aparm)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  旧参数名到新参数名的转换表。启动时会查找旧参数值，
  若新参数尚未保存，则复制旧值并保存新参数。
  
  即便旧参数已移除，该机制仍可工作，前提是旧 k_param 索引未被移除。
  
  下表第二列为旧对象在 var_info[] 中的索引。顶层参数应为 0。
 */
static const AP_Param::ConversionInfo conversion_table[] = {
    { Parameters::k_param_fence_minalt,       0,     AP_PARAM_INT16, "FENCE_ALT_MIN"},
    { Parameters::k_param_fence_maxalt,       0,     AP_PARAM_INT16, "FENCE_ALT_MAX"},
    { Parameters::k_param_fence_retalt,       0,     AP_PARAM_INT16, "FENCE_RET_ALT"},
    { Parameters::k_param_fence_ret_rally,    0,      AP_PARAM_INT8, "FENCE_RET_RALLY"},
    { Parameters::k_param_fence_autoenable,   0,      AP_PARAM_INT8, "FENCE_AUTOENABLE"},
};

struct RCConversionInfo {
    uint16_t old_key; // k_param_*
    uint32_t old_group_element; // 旧对象中的索引
    RC_Channel::AUX_FUNC fun; // 新功能
};

static const RCConversionInfo rc_option_conversion[] = {
    { Parameters::k_param_flapin_channel_old, 0, RC_Channel::AUX_FUNC::FLAP},
    { Parameters::k_param_g2, 968, RC_Channel::AUX_FUNC::SOARING},
#if AP_FENCE_ENABLED
    { Parameters::k_param_fence_channel, 0, RC_Channel::AUX_FUNC::FENCE},
#endif
#if AP_MISSION_ENABLED
    { Parameters::k_param_reset_mission_chan, 0, RC_Channel::AUX_FUNC::MISSION_RESET},
#endif
#if HAL_PARACHUTE_ENABLED
    { Parameters::k_param_parachute_channel, 0, RC_Channel::AUX_FUNC::PARACHUTE_RELEASE},
#endif
    { Parameters::k_param_fbwa_tdrag_chan, 0, RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER},
    { Parameters::k_param_reset_switch_chan, 0, RC_Channel::AUX_FUNC::MODE_SWITCH_RESET},
};

void Plane::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    // 设置 SRV_Channels 默认值
    g2.servo_channels.set_default_function(CH_1, SRV_Channel::k_aileron);
    g2.servo_channels.set_default_function(CH_2, SRV_Channel::k_elevator);
    g2.servo_channels.set_default_function(CH_3, SRV_Channel::k_throttle);
    g2.servo_channels.set_default_function(CH_4, SRV_Channel::k_rudder);
        
    SRV_Channels::upgrade_parameters();

#if HAL_QUADPLANE_ENABLED
    if (quadplane.enable) {
        // QuadPlane 需要更高的循环频率
        AP_Param::set_default_by_name("SCHED_LOOP_RATE", 300);
    }
#endif

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_PLANE);

    // 将通道参数转换为 RCx_OPTION
    for (uint8_t i=0; i<ARRAY_SIZE(rc_option_conversion); i++) {
        AP_Int8 chan_param;
        AP_Param::ConversionInfo info {rc_option_conversion[i].old_key, rc_option_conversion[i].old_group_element, AP_PARAM_INT8, nullptr};
        if (AP_Param::find_old_parameter(&info, &chan_param) && chan_param.get() > 0) {
            RC_Channel *chan = rc().channel(chan_param.get() - 1);
            if (chan != nullptr && !chan->option.configured()) {
                chan->option.set_and_save((int16_t)rc_option_conversion[i].fun); // 保存新参数
            }
        }
    }


// PARAMETER_CONVERSION - Added: March 2021 for ArduPlane-4.1
#if AP_FENCE_ENABLED
    enum ap_var_type ptype_fence_type;
    AP_Int8 *fence_type_new = (AP_Int8*)AP_Param::find("FENCE_TYPE", &ptype_fence_type);
    if (fence_type_new && !fence_type_new->configured()) {
        // 找到新参数且未配置时，尝试升级高度围栏
        int8_t fence_type_new_val = AC_FENCE_TYPE_POLYGON;
        AP_Int16 fence_alt_min_old;
        AP_Param::ConversionInfo fence_alt_min_info_old = {
            Parameters::k_param_fence_minalt,
            0,
            AP_PARAM_INT16,
            nullptr
        };
        if (AP_Param::find_old_parameter(&fence_alt_min_info_old, &fence_alt_min_old)) {
            if (fence_alt_min_old.configured()) {
                //
                fence_type_new_val |= AC_FENCE_TYPE_ALT_MIN;
            }
        }

        AP_Int16 fence_alt_max_old;
        AP_Param::ConversionInfo fence_alt_max_info_old = {
            Parameters::k_param_fence_maxalt,
            0,
            AP_PARAM_INT16,
            nullptr
        };
        if (AP_Param::find_old_parameter(&fence_alt_max_info_old, &fence_alt_max_old)) {
            if (fence_alt_max_old.configured()) {
                fence_type_new_val |= AC_FENCE_TYPE_ALT_MAX;
            }
        }

        fence_type_new->set_and_save((int8_t)fence_type_new_val);
    }

    AP_Int8 fence_action_old;
    AP_Param::ConversionInfo fence_action_info_old = {
        Parameters::k_param_fence_action,
        0,
        AP_PARAM_INT8,
        "FENCE_ACTION"
    };
    if (AP_Param::find_old_parameter(&fence_action_info_old, &fence_action_old)) {
        enum ap_var_type ptype;
        AP_Int8 *fence_action_new = (AP_Int8*)AP_Param::find(&fence_action_info_old.new_name[0], &ptype);
        AC_Fence::Action fence_action_new_val;
        if (fence_action_new && !fence_action_new->configured()) {
            switch(fence_action_old.get()) {
                case 0: // FENCE_ACTION_NONE
                case 2: // FENCE_ACTION_REPORT_ONLY
                default:
                    fence_action_new_val = AC_Fence::Action::REPORT_ONLY;
                    break;
                case 1: // FENCE_ACTION_GUIDED
                    fence_action_new_val = AC_Fence::Action::GUIDED;
                    break;
                case 3: // FENCE_ACTION_GUIDED_THR_PASS
                    fence_action_new_val = AC_Fence::Action::GUIDED_THROTTLE_PASS;
                    break;
                case 4: // FENCE_ACTION_RTL
                    fence_action_new_val = AC_Fence::Action::RTL_AND_LAND;
                    break;
            }
            fence_action_new->set_and_save((int8_t)fence_action_new_val);
            
            // 同时升级新的围栏启用参数
            enum ap_var_type ptype_fence_enable;
            AP_Int8 *fence_enable = (AP_Int8*)AP_Param::find("FENCE_ENABLE", &ptype_fence_enable);
            // 若存在围栏数量且旧围栏动作不为 0，则认为使用过围栏
            AC_Fence *ap_fence = AP::fence();
            bool fences_exist = false;
            if (ap_fence) {
                // 若围栏库存在，尝试读取围栏数量
                fences_exist = ap_fence->polyfence().total_fence_count() > 0;
            }
            
            bool fences_used = fence_action_old.get() != 0;
            if (fence_enable && !fence_enable->configured()) {
                // 围栏启用参数存在，按情况设置
                fence_enable->set_and_save(fences_exist && fences_used);
            }
        }
    }
#endif // AP_FENCE_ENABLED

#if AP_TERRAIN_AVAILABLE
    g.terrain_follow.convert_parameter_width(AP_PARAM_INT8);
#endif

    g.use_reverse_thrust.convert_parameter_width(AP_PARAM_INT16);

#if AP_AIRSPEED_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2022
    {
        const uint16_t old_key = g.k_param_airspeed;
        const uint16_t old_index = 0;       // 旧参数在树中的索引
        AP_Param::convert_class(old_key, &airspeed, airspeed.var_info, old_index, true);
    }
#endif

#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
#if HAL_INS_NUM_HARMONIC_NOTCH_FILTERS > 1
    if (!ins.harmonic_notches[1].params.enabled()) {
        // 固定陷波滤波器参数转换（移到 INS_HNTC2），适用于 4.2.x
        const AP_Param::ConversionInfo notchfilt_conversion_info[] {
            { Parameters::k_param_ins, 101, AP_PARAM_INT8,  "INS_HNTC2_ENABLE" },
            { Parameters::k_param_ins, 293, AP_PARAM_FLOAT, "INS_HNTC2_ATT" },
            { Parameters::k_param_ins, 357, AP_PARAM_FLOAT, "INS_HNTC2_FREQ" },
            { Parameters::k_param_ins, 421, AP_PARAM_FLOAT, "INS_HNTC2_BW" },
        };
        AP_Param::convert_old_parameters(&notchfilt_conversion_info[0], ARRAY_SIZE(notchfilt_conversion_info));
        AP_Param::set_default_by_name("INS_HNTC2_MODE", 0);
        AP_Param::set_default_by_name("INS_HNTC2_HMNCS", 1);
    }
#endif // HAL_INS_NUM_HARMONIC_NOTCH_FILTERS
#endif  // AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED

    // PARAMETER_CONVERSION - Added: Mar-2022
#if AP_FENCE_ENABLED
    AP_Param::convert_class(g.k_param_fence, &fence, fence.var_info, 0, true);
#endif

    // PARAMETER_CONVERSION - Added: July-2025 for ArduPilot-4.7
#if AP_RPM_ENABLED
    AP_Param::convert_class(g.k_param_rpm_sensor_old, &rpm_sensor, rpm_sensor.var_info, 0, true, true);
#endif

    // PARAMETER_CONVERSION - Added: Dec 2023
    // 将 _CM（厘米）参数转换为米，将 _CD（0.01 度）参数转换为米
    g.pitch_trim.convert_centi_parameter(AP_PARAM_INT16);
    aparm.airspeed_cruise.convert_centi_parameter(AP_PARAM_INT32);
    aparm.min_groundspeed.convert_centi_parameter(AP_PARAM_INT32);
    g.RTL_altitude.convert_centi_parameter(AP_PARAM_INT32);
    g.cruise_alt_floor.convert_centi_parameter(AP_PARAM_INT16);
    aparm.pitch_limit_max.convert_centi_parameter(AP_PARAM_INT16);
    aparm.pitch_limit_min.convert_centi_parameter(AP_PARAM_INT16);
    aparm.roll_limit.convert_centi_parameter(AP_PARAM_INT16);

    landing.convert_parameters();

    static const AP_Param::G2ObjectConversion g2_conversions[] {
    // PARAMETER_CONVERSION - Added: Oct-2021
#if HAL_EFI_ENABLED
        { &efi, efi.var_info, 22 },
#endif
#if AP_STATS_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024 for Plane-4.6
        { &stats, stats.var_info, 5 },
#endif
#if AP_SCRIPTING_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024 for Plane-4.6
        { &scripting, scripting.var_info, 14 },
#endif
#if AP_GRIPPER_ENABLED
    // PARAMETER_CONVERSION - Added: Feb-2024 for Plane-4.6
        { &gripper, gripper.var_info, 12 },
#endif
    };

    AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

    // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

    static const AP_Param::TopLevelObjectConversion toplevel_conversions[] {
#if AP_SERIALMANAGER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024 for Plane-4.6
        { &serial_manager, serial_manager.var_info, Parameters::k_param_serial_manager_old },
#endif
    };

    AP_Param::convert_toplevel_objects(toplevel_conversions, ARRAY_SIZE(toplevel_conversions));

#if HAL_GCS_ENABLED
    // Move parameters into new MAV_ parameter namespace
    // PARAMETER_CONVERSION - Added: Mar-2025 for ArduPilot-4.7
    {
        static const AP_Param::ConversionInfo gcs_conversion_info[] {
            { Parameters::k_param_sysid_this_mav_old, 0, AP_PARAM_INT16,  "MAV_SYSID" },
            { Parameters::k_param_sysid_my_gcs_old, 0, AP_PARAM_INT16, "MAV_GCS_SYSID" },
            { Parameters::k_param_g2,  4, AP_PARAM_INT8, "MAV_OPTIONS" },
            { Parameters::k_param_telem_delay_old,  0, AP_PARAM_INT8, "MAV_TELEM_DELAY" },
        };
        AP_Param::convert_old_parameters(&gcs_conversion_info[0], ARRAY_SIZE(gcs_conversion_info));
    }
#endif  // HAL_GCS_ENABLED
}
