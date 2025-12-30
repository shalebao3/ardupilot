#pragma once

#define AP_PARAM_VEHICLE_NAME plane

#include <AP_Common/AP_Common.h>

// 全局参数类
//
class Parameters {
public:

    /*
     *  k_format_version 的值决定已有 EEPROM 数据是否有效。仅在以下情况下
     *  才应该修改该值：
     *
     *  1) 现有 EEPROM 参数含义发生变化
     *
     *  2) 现有 k_param_* 枚举值发生变化
     *
     *  新增参数通常不需要修改 k_format_version，除非特殊情况。若随意修改，
     *  所有 ArduPlane 用户都需要重新加载参数，这应极其少见。
     *
     *  判断 k_param_* 是否变更时，请依据 C++ 枚举规则计算相邻枚举值。
     *  若不了解规则，请先咨询其他开发者。
     */

    //////////////////////////////////////////////////////////////////
    // 注意：未充分理解以上说明前请勿修改该值。如不确定，请咨询其他开发者！
    static const uint16_t k_format_version = 13;
    //////////////////////////////////////////////////////////////////

    enum class ThrFailsafe {
        Disabled    = 0,
        Enabled     = 1,
        EnabledNoFS = 2
    };

    enum {
        // 布局版本号，始终为 key 0
        //
        k_param_format_version = 0,
        k_param_software_type, // 未使用
        k_param_num_resets, // 未使用
        k_param_NavEKF2,
        k_param_g2,
        k_param_avoidance_adsb,
        k_param_landing,
        k_param_NavEKF3,
        k_param_can_mgr,
        k_param_osd,

        // 杂项
        //
        k_param_auto_trim      = 10, // 未使用
        k_param_log_bitmask_old,  // 未使用
        k_param_pitch_trim,
        k_param_mix_mode,
        k_param_reverse_elevons, // 未使用
        k_param_reverse_ch1_elevon, // 未使用
        k_param_reverse_ch2_elevon, // 未使用
        k_param_flap_1_percent,
        k_param_flap_1_speed,
        k_param_flap_2_percent,
        k_param_flap_2_speed,
        k_param_reset_switch_chan, // 未使用 - 已移到 RC 选项
        k_param_manual_level, // 未使用
        k_param_land_pitch_cd,  // 未使用 - 已移到 AP_Landing
        k_param_ins_old,            // 已废弃，下次 EEPROM 版本变更时移除
        k_param_stick_mixing,
        k_param_reset_mission_chan, // 未使用 - 已移到 RC 选项
        k_param_land_flare_alt, // 未使用 - 已移到 AP_Landing
        k_param_land_flare_sec, // 未使用 - 已移到 AP_Landing
        k_param_crosstrack_min_distance, // 未使用
        k_param_rudder_steer, // 未使用
        k_param_throttle_nudge,
        k_param_alt_offset,
        k_param_ins,                // libraries/AP_InertialSensor 变量
        k_param_takeoff_throttle_min_speed,
        k_param_takeoff_throttle_min_accel,
        k_param_takeoff_heading_hold, // 未使用
        k_param_level_roll_limit,
        k_param_hil_servos_unused,  // 未使用
        k_param_vtail_output, // 未使用
        k_param_nav_controller, // 未使用
        k_param_elevon_output, // 未使用
        k_param_att_controller, // 未使用
        k_param_mixing_gain,
        k_param_scheduler,
        k_param_relay,
        k_param_takeoff_throttle_delay,
        k_param_mode_takeoff, // 曾为 skip_gyro_cal
        k_param_auto_fbw_steer, // 未使用
        k_param_waypoint_max_radius,
        k_param_ground_steer_alt,        
        k_param_ground_steer_dps,
        k_param_rally_limit_km_old, // 已不再使用，仅保留索引
        k_param_hil_err_limit_unused,  // 未使用
        k_param_sonar_old, // 未使用
        k_param_log_bitmask,
        k_param_BoardConfig,
        k_param_rssi_range,     // 未使用，已由 rssi_ 库参数替代
        k_param_flapin_channel_old,  // 未使用，已移到 RC_OPTION
        k_param_flaperon_output, // 未使用
        k_param_gps,
        k_param_autotune_level,
        k_param_rally,
        k_param_serial0_baud,           // 已废弃
        k_param_serial1_baud,           // 已废弃
        k_param_serial2_baud,           // 已废弃
        k_param_takeoff_tdrag_elevator,
        k_param_takeoff_tdrag_speed1,
        k_param_takeoff_rotate_speed,
        k_param_takeoff_throttle_slewrate,
        k_param_takeoff_throttle_max,
        k_param_rangefinder,
        k_param_terrain,
        k_param_terrain_follow,
        k_param_stab_pitch_down_cd_old, // 已废弃
        k_param_alt_slope_min,
        k_param_stab_pitch_down,
        k_param_terrain_lookahead,
        k_param_fbwa_tdrag_chan, // 未使用 - 已移到 RC 选项
        k_param_rangefinder_landing,
        k_param_land_flap_percent,  // 未使用 - 已移到 AP_Landing
        k_param_takeoff_flap_percent,
        k_param_flap_slewrate,
        k_param_rtl_autoland,
        k_param_override_channel,
        k_param_stall_prevention,
        k_param_optflow,
        k_param_cli_enabled_old, // 未使用 - CLI 已移除
        k_param_trim_rc_at_start, // 未使用
        k_param_hil_mode_unused,  // 未使用
        k_param_land_disarm_delay,  // 未使用 - 已移到 AP_Landing
        k_param_alt_slope_max_height,
        k_param_rudder_only,
        k_param_gcs3_unused,               // ArduPilot-4.7 中未使用
        k_param_gcs_pid_mask,
        k_param_crash_detection_enable,
        k_param_land_abort_throttle_enable, // 未使用 - 已移到 AP_Landing
        k_param_rssi = 97,
        k_param_rpm_sensor_old, // 未使用 - 已移到 vehicle
        k_param_parachute,
        k_param_arming = 100,
        k_param_parachute_channel, // 未使用 - 已移到 RC 选项
        k_param_crash_accel_threshold,
        k_param_override_safety, // 未使用
        k_param_land_throttle_slewrate, // 104 未使用 - 已移到 AP_Landing

        // 105：附加参数
        k_param_fence_retalt = 105,
        k_param_fence_autoenable,
        k_param_fence_ret_rally,
        k_param_q_attitude_control,
        k_param_takeoff_pitch_limit_reduction_sec,

        // 110：遥测控制
        //
        k_param_gcs0_unused = 110,         // ArduPilot-4.7 中未使用
        k_param_gcs1_unused,               // ArduPilot-4.7 中未使用
        k_param_sysid_this_mav_old,
        k_param_sysid_my_gcs_old,
        k_param_serial1_baud_old,   // 已废弃
        k_param_telem_delay_old,
        k_param_serial0_baud_old,   // 已废弃
        k_param_gcs2_unused,               // ArduPilot-4.7 中未使用
        k_param_serial2_baud_old,   // 已废弃
        k_param_serial2_protocol,   // 已废弃

        // 120：电传控制
        //
        k_param_airspeed_min = 120,
        k_param_airspeed_max,
        k_param_cruise_alt_floor,
        k_param_flybywire_elev_reverse,
        k_param_alt_control_algorithm, // 未使用
        k_param_flybywire_climb_rate,
        k_param_acro_roll_rate,
        k_param_acro_pitch_rate,
        k_param_acro_locking,
        k_param_use_reverse_thrust = 129,

        //
        // 130：传感器参数
        //
        k_param_imu = 130,  // 未使用
        k_param_altitude_mix, // 已废弃

        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_battery_monitoring, // 未使用
        k_param_volt_div_ratio,     // 未使用
        k_param_curr_amp_per_volt,  // 未使用
        k_param_input_voltage, // 已废弃，可删除
        k_param_pack_capacity,      // 未使用
        k_param_sonar_enabled_old,  // 未使用
        k_param_ahrs,  // AHRS 组
        k_param_barometer,   // 气压计地面校准
        k_param_airspeed,           // 仅用于参数转换；AP_Airspeed 参数已移到 AP_Vehicle
        k_param_curr_amp_offset,
        k_param_NavEKF,  // 已废弃 - 移除
        k_param_mission, // 任务库
        k_param_serial_manager_old, // 串口管理库
        k_param_NavEKF2_old,  // 已废弃 - 移除
        k_param_land_pre_flare_alt, // 未使用 - 已移到 AP_Landing
        k_param_land_pre_flare_airspeed = 149,  // 未使用 - 已移到 AP_Landing

        //
        // 150：导航参数
        //
        k_param_crosstrack_gain = 150, // 未使用
        k_param_crosstrack_entry_angle, // 未使用
        k_param_roll_limit,
        k_param_pitch_limit_max,
        k_param_pitch_limit_min,
        k_param_airspeed_cruise,
        k_param_RTL_altitude,
        k_param_inverted_flight_ch_unused, // 未使用
        k_param_min_groundspeed,
        k_param_crosstrack_use_wind, // 未使用


        //
        // 相机与云台参数
        //
        k_param_camera = 160,
        k_param_camera_mount,
        k_param_camera_mount2,      // 未使用
        k_param_adsb,
        k_param_notify,
        k_param_land_pre_flare_sec = 165,   // 未使用 - 已移到 AP_Landing

        //
        // 电池监测参数
        //
        k_param_battery = 166,
        k_param_rssi_pin,               // 未使用，已由 rssi_ 库参数替代 - 167
        k_param_battery_volt_pin,       // 未使用 - 168
        k_param_battery_curr_pin,       // 未使用 - 169

        //
        // 170：遥控设置 - 现已全部未使用
        //
        k_param_rc_1_old = 170,
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_9_old,
        k_param_rc_10_old,
        k_param_rc_11_old,

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_fs_action_short,
        k_param_fs_action_long,
        k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,
        k_param_throttle_suppress_manual,
        k_param_throttle_passthru_stabilize,
        k_param_rc_12_old,
        k_param_fs_batt_voltage, // 未使用 - 已移到 AP_BattMonitor
        k_param_fs_batt_mah,     // 未使用 - 已移到 AP_BattMonitor
        k_param_fs_timeout_short_unused, // 未使用
        k_param_fs_timeout_long,
        k_param_rc_13_old,
        k_param_rc_14_old,
        k_param_tuning,

        //
        // 200：前馈增益
        //
        k_param_kff_pitch_compensation = 200, // 未使用
        k_param_kff_rudder_mix,
        k_param_kff_pitch_to_throttle, // 未使用
        k_param_kff_throttle_to_pitch,
        k_param_scaling_speed,
        k_param_quadplane,
        k_param_rtl_radius,
        k_param_land_then_servos_neutral,   // 未使用 - 已移到 AP_Landing
        k_param_rc_15_old,
        k_param_rc_16_old,

        //
        // 210：飞行模式
        //
        k_param_flight_mode_channel = 210,
        k_param_flight_mode1,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_initial_mode,
        k_param_land_slope_recalc_shallow_threshold,    // 未使用 - 已移到 AP_Landing
        k_param_land_slope_recalc_steep_threshold_to_abort, // 未使用 - 已移到 AP_Landing

        //
        // 220：航点数据
        //
        k_param_waypoint_mode = 220, // 未使用
        k_param_command_total,  // 未使用
        k_param_command_index,  // 未使用
        k_param_waypoint_radius,
        k_param_loiter_radius,
        k_param_fence_action,
        k_param_fence_total,
        k_param_fence_channel, // 未使用 - 已移到 RC 选项
        k_param_fence_minalt,
        k_param_fence_maxalt,

        // 其他对象
        k_param_sitl = 230,
        k_param_afs,
        k_param_rollController,
        k_param_pitchController,
        k_param_yawController,
        k_param_L1_controller,
        k_param_rcmap,
        k_param_TECS_controller,
        k_param_rally_total_old,  // 未使用
        k_param_steerController,

        //
        // 240：PID 控制器
        k_param_pidNavRoll = 240, // 未使用
        k_param_pidServoRoll, // 未使用
        k_param_pidServoPitch, // 未使用
        k_param_pidNavPitchAirspeed, // 未使用
        k_param_pidServoRudder, // 未使用
        k_param_pidTeThrottle, // 未使用
        k_param_pidNavPitchAltitude, // 未使用
        k_param_pidWheelSteer, // 未使用

        k_param_mixing_offset,
        k_param_dspoiler_rud_rate,
        k_param_airspeed_stall,

        k_param_logger = 253, // 日志组

        // 254,255：保留

        k_param_vehicle = 257, // 机型通用参数块
        k_param_gcs4_unused,               // ArduPilot-4.7 中未使用
        k_param_gcs5_unused,               // ArduPilot-4.7 中未使用
        k_param_gcs6_unused,               // ArduPilot-4.7 中未使用
        k_param_fence,         // 机型围栏 - 未使用
        k_param_acro_yaw_rate,
        k_param_takeoff_throttle_max_t,
        k_param_autotune_options,
        k_param_takeoff_throttle_min,
        k_param_takeoff_options,
        k_param_takeoff_throttle_idle,

        k_param_pullup = 270,
        k_param_quicktune,
        k_param_mode_autoland,
        k_param__gcs,

    };

    AP_Int16 format_version;

    AP_Enum<RtlAutoland> rtl_autoland;

    AP_Int8  crash_accel_threshold;

    // 前馈增益
    //
    AP_Float kff_rudder_mix;
    AP_Float kff_pitch_to_throttle;
    AP_Float kff_throttle_to_pitch;
    AP_Float ground_steer_alt;
    AP_Int16 ground_steer_dps;
    AP_Float stab_pitch_down;

    // 用于速度缩放的速度
    AP_Float scaling_speed;

    // 航点
    //
    AP_Int16 waypoint_radius;
    AP_Int16 waypoint_max_radius;
    AP_Int16 rtl_radius;

    // 电传
    //
    AP_Int8 flybywire_elev_reverse;
    AP_Int8 flybywire_climb_rate;

    // 油门
    //
    AP_Int8 throttle_suppress_manual;
    AP_Int8 throttle_passthru_stabilize;
    AP_Enum<ThrFailsafe> throttle_fs_enabled;
    AP_Int16 throttle_fs_value;
    AP_Int8 throttle_nudge;
    AP_Int32 use_reverse_thrust;

    // 失效保护
    AP_Int8 fs_action_short;
    AP_Int8 fs_action_long;
    AP_Float fs_timeout_long;
    AP_Int8 gcs_heartbeat_fs_enabled;

    // 飞行模式
    //
    AP_Int8 flight_mode_channel;
    AP_Int8 flight_mode1;
    AP_Int8 flight_mode2;
    AP_Int8 flight_mode3;
    AP_Int8 flight_mode4;
    AP_Int8 flight_mode5;
    AP_Int8 flight_mode6;
    AP_Int8 initial_mode;

    // 导航机动限制
    //
    AP_Int16 alt_offset;
    AP_Int16 acro_roll_rate;
    AP_Int16 acro_pitch_rate;
    AP_Int16 acro_yaw_rate;
    AP_Int8  acro_locking;

    // 杂项
    //
    AP_Int8 rudder_only;
    AP_Float mixing_gain;
    AP_Int16 mixing_offset;
    AP_Int16 dspoiler_rud_rate;
    AP_Int32 log_bitmask;
    AP_Float RTL_altitude;
    AP_Float pitch_trim;
    AP_Float cruise_alt_floor;

    AP_Int8 flap_1_percent;
    AP_Int8 flap_1_speed;
    AP_Int8 flap_2_percent;
    AP_Int8 flap_2_speed;
    AP_Int8 takeoff_flap_percent;
    AP_Enum<StickMixing> stick_mixing;
    AP_Float takeoff_throttle_min_speed;
    AP_Float takeoff_throttle_min_accel;
    AP_Int8 takeoff_throttle_delay;
    AP_Int8 takeoff_tdrag_elevator;
    AP_Float takeoff_tdrag_speed1;
    AP_Float takeoff_rotate_speed;
    AP_Int8 takeoff_throttle_slewrate;
    AP_Float takeoff_pitch_limit_reduction_sec;
    AP_Int8 level_roll_limit;
#if AP_TERRAIN_AVAILABLE
    AP_Int32 terrain_follow;
    AP_Int16 terrain_lookahead;
#endif
    AP_Int16 alt_slope_min;
    AP_Float alt_slope_max_height;
    AP_Int8 rangefinder_landing;
    AP_Int8 flap_slewrate;
#if HAL_WITH_IO_MCU
    AP_Int8 override_channel;
#endif
    AP_Int16 gcs_pid_mask;
};

/*
  第二组参数，避免超过 256 个顶层键
 */
class ParametersG2 {
public:
    ParametersG2(void);

    // 保存参数信息的 var_info
    static const struct AP_Param::GroupInfo var_info[];

    // 在全部编译禁用时保持编译通过
    uint8_t unused_integer;

    // 按键上报库
#if HAL_BUTTON_ENABLED
    AP_Button *button_ptr;
#endif

#if AP_ICENGINE_ENABLED
    // 内燃机控制
    AP_ICEngine ice_control;
#endif

    // RC 输入通道
    RC_Channels_Plane rc_channels;
    
    // 舵机输出范围控制
    SRV_Channels servo_channels;

#if HAL_SOARING_ENABLED
    // ArduSoar 参数
    SoaringController soaring_controller;
#endif

    // 双电机尾座式方向舵到差动推力缩放：0-100%
    AP_Int8 rudd_dt_gain;

    // 手动直通的通道掩码
    AP_Int32 manual_rc_mask;

    // Home 复位高度阈值
    AP_Int8 home_reset_threshold;

    AP_Int32 flight_options;

    AP_Int16 waypoint_climb_slope_height_min;

    AP_Int8 takeoff_throttle_accel_count;
    AP_Int8 takeoff_timeout;

#if AP_LANDINGGEAR_ENABLED
    AP_LandingGear landing_gear;
#endif

#if AC_PRECLAND_ENABLED
    AC_PrecLand precland;
#endif

    // 鸭式襟翼权重
    AP_Int8 crow_flap_weight_outer;
    AP_Int8 crow_flap_weight_inner;
    AP_Int8 crow_flap_options;
    AP_Int8 crow_flap_aileron_matching;

    // 前向油门电压补偿
    class FWD_BATT_CMP {
    public:
        // 计算油门缩放以补偿电池电压下降
        void update();

        // 将油门缩放应用到最小/最大限制
        void apply_min_max(int8_t &min_throttle, int8_t &max_throttle) const;

        // 将油门缩放应用到油门需求
        float apply_throttle(float throttle) const;

        AP_Float batt_voltage_max;
        AP_Float batt_voltage_min;
        AP_Float batt_voltage_throttle_cutoff;
        AP_Int8  batt_idx;

    private:
        bool enabled;
        float ratio;
    } fwd_batt_cmp;


#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // 引导模式偏航航向 PID
    AC_PID guidedHeading{5000.0,  0.0,   0.0, 0 ,  10.0,   5.0,  5.0 ,  5.0  , 0.0};
#endif

    AP_Float guided_timeout;

#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    AP_Follow follow;
#endif

    AP_Float        fs_ekf_thresh;

    // RTL 最小初始爬升
    AP_Int16        rtl_climb_min;

    AP_Int8         man_expo_roll;
    AP_Int8         man_expo_pitch;
    AP_Int8         man_expo_rudder;

    AP_Int32        oneshot_mask;
    
    AP_Int8         axis_bitmask; // 需自动调参的轴

#if AP_RANGEFINDER_ENABLED
    // 用于降落的测距仪朝向
    AP_Int8 rangefinder_land_orient;
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    AP_SystemID systemid;
#endif
};

extern const AP_Param::Info var_info[];
