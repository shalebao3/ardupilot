/*
   Lead developer: Andrew Tridgell & Tom Pittenger

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.com for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

////////////////////////////////////////////////////////////////////////////////
// 头文件包含
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega 向量/矩阵数学库
#include <AP_InertialSensor/AP_InertialSensor.h> // 惯性传感器库
#include <AP_AccelCal/AP_AccelCal.h>                // 加速度计校准接口与数学工具
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM 库
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>     // 测距仪库
#include <Filter/Filter.h>                     // 滤波库
#include <AP_Camera/AP_Camera.h>          // 照相或视频相机
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Beacon/AP_Beacon.h>

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <APM_Control/APM_Control.h>
#include <APM_Control/AP_AutoTune.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink 地面站定义
#include <AP_Mount/AP_Mount.h>           // 相机/天线云台
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega 磁偏角辅助库
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // 主循环调度器
#include <AP_Scheduler/PerfInfo.h>                  // 循环性能监控

#include <AP_Navigation/AP_Navigation.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC 输入映射库

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>     // 任务指令库

#include <AP_Soaring/AP_Soaring.h>
#include <AP_BattMonitor/AP_BattMonitor.h> // 电池监测库

#include <AP_Arming/AP_Arming.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_OSD/AP_OSD.h>

#include <AP_Rally/AP_Rally.h>

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // 光流库
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Avoidance/AP_Avoidance_config.h>      // “ADSB” 避障库
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_LandingGear/AP_LandingGear.h>     // 起落架库
#include <AP_Follow/AP_Follow.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Plane.h"
#endif

#include <AC_PrecLand/AC_PrecLand_config.h>
#if AC_PRECLAND_ENABLED
 # include <AC_PrecLand/AC_PrecLand.h>
#endif

#include "GCS_MAVLink_Plane.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include <AP_Tuning/AP_Tuning_config.h>
#if AP_TUNING_ENABLED
#include "tuning.h"
#endif

// 配置
#include "config.h"

#if AP_ADVANCEDFAILSAFE_ENABLED
#include "afs_plane.h"
#endif

// 本地模块
#include "defines.h"
#include "mode.h"

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "RC_Channel_Plane.h"     // RC 通道库
#include "Parameters.h"
#if AP_ADSB_AVOIDANCE_ENABLED
#include "avoidance_adsb.h"
#endif  // AP_ADSB_AVOIDANCE_ENABLED
#include "AP_Arming_Plane.h"
#include "pullup.h"
#include "systemid.h"

/*
  APM:Plane 主类
 */
class Plane : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Plane;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Plane;
    friend class QuadPlane;
    friend class QAutoTune;
    friend class AP_Tuning_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class AP_Avoidance_Plane;
    friend class GCS_Plane;
    friend class RC_Channel_Plane;
    friend class RC_Channels_Plane;
    friend class Tailsitter;
    friend class Tiltrotor;
    friend class SLT_Transition;
    friend class Tailsitter_Transition;
    friend class VTOL_Assist;

    friend class Mode;
    friend class ModeCircle;
    friend class ModeStabilize;
    friend class ModeTraining;
    friend class ModeAcro;
    friend class ModeFBWA;
    friend class ModeFBWB;
    friend class ModeCruise;
    friend class ModeAutoTune;
    friend class ModeAuto;
    friend class ModeRTL;
    friend class ModeLoiter;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeInitializing;
    friend class ModeManual;
    friend class ModeQStabilize;
    friend class ModeQHover;
    friend class ModeQLoiter;
    friend class ModeQLand;
    friend class ModeQRTL;
    friend class ModeQAcro;
    friend class ModeQAutotune;
    friend class ModeTakeoff;
    friend class ModeThermal;
    friend class ModeLoiterAltQLand;
#if MODE_AUTOLAND_ENABLED
    friend class ModeAutoLand;
#endif
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Plane;
#endif
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    friend class GliderPullup;
#endif
#if AP_PLANE_SYSTEMID_ENABLED
    friend class AP_SystemID;
#endif

    Plane(void);

private:

    // 传递给多个库的关键飞行器参数
    AP_FixedWing aparm;

    // 全局参数都包含在 g 与 g2 中
    Parameters g;
    ParametersG2 g2;

    // 输入通道映射
    RCMapper rcmap;

    // 主要输入通道
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;
    RC_Channel *channel_flap;
    RC_Channel *channel_airbrake;

    // 基于俯仰缩放的横滚限制
    int32_t roll_limit_cd;
    float pitch_limit_min;

    // 飞行模式便捷数组
    AP_Int8 *flight_modes = &g.flight_mode1;
    const uint8_t num_flight_modes = 6;

#if AP_RANGEFINDER_ENABLED
    AP_FixedWing::Rangefinder_State rangefinder_state;

    /*
      用于降落的测距仪朝向
     */
    Rotation rangefinder_orientation(void) const {
        return Rotation(g2.rangefinder_land_orient.get());
    }

#endif

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    struct {
        // 允许外部提供离地高度估计
        float hagl;
        uint32_t last_update_ms;
        uint32_t timeout_ms;
    } external_hagl;
    bool get_external_HAGL(float &height_agl);
    void handle_external_hagl(const mavlink_command_int_t &packet);
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

    float get_landing_height(bool &using_rangefinder);

    AP_TECS TECS_controller{ahrs, aparm, landing, MASK_LOG_TECS};
    AP_L1_Control L1_controller{ahrs, &TECS_controller};

    // 姿态到舵机控制器
    AP_RollController rollController{aparm};
    AP_PitchController pitchController{aparm};
    AP_YawController yawController{aparm};
    AP_SteerController steerController{};

    // 训练模式
    bool training_manual_roll;  // 用户拥有手动横滚控制
    bool training_manual_pitch; // 用户拥有手动俯仰控制

    // 引导模式下油门是否直通
    bool guided_throttle_passthru;

    // 是否正在校准：用于在气压计与空速校准期间向外部失效保护板发送心跳
    bool in_calibration;

    // 是否处于长失效保护且在 TAKEOFF 模式下延后执行，直到到达最低平飞高度
    bool long_failsafe_pending;

    // 地面站选择
    GCS_Plane _gcs; // 避免直接使用；请使用 gcs()
    GCS_Plane &gcs() { return _gcs; }

    // 选择的导航控制器
    AP_Navigation *nav_controller = &L1_controller;

    // 相机
#if AP_CAMERA_ENABLED
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

#if AP_OPTICALFLOW_ENABLED
    // 光流传感器
    AP_OpticalFlow optflow;
#endif

#if HAL_RALLY_ENABLED
    // 备降点
    AP_Rally rally;
#endif

#if AC_PRECLAND_ENABLED
    void precland_update(void);
#endif

    // 返回备降点或 Home 的位置；若 HAL_RALLY_ENABLED 为 false，则返回 Home
    Location calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt_amsl_cm) const;

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif

    ModeCircle mode_circle;
    ModeStabilize mode_stabilize;
    ModeTraining mode_training;
    ModeAcro mode_acro;
    ModeFBWA mode_fbwa;
    ModeFBWB mode_fbwb;
    ModeCruise mode_cruise;
    ModeAutoTune mode_autotune;
    ModeAuto mode_auto;
    ModeRTL mode_rtl;
    ModeLoiter mode_loiter;
#if HAL_ADSB_ENABLED
    ModeAvoidADSB mode_avoidADSB;
#endif
    ModeGuided mode_guided;
    ModeInitializing mode_initializing;
    ModeManual mode_manual;
#if HAL_QUADPLANE_ENABLED
    ModeQStabilize mode_qstabilize;
    ModeQHover mode_qhover;
    ModeQLoiter mode_qloiter;
    ModeQLand mode_qland;
    ModeQRTL mode_qrtl;
    ModeQAcro mode_qacro;
    ModeLoiterAltQLand mode_loiter_qland;
#if QAUTOTUNE_ENABLED
    ModeQAutotune mode_qautotune;
#endif  // QAUTOTUNE_ENABLED
#endif  // HAL_QUADPLANE_ENABLED
    ModeTakeoff mode_takeoff;
#if MODE_AUTOLAND_ENABLED
    ModeAutoLand mode_autoland;
#endif
#if HAL_SOARING_ENABLED
    ModeThermal mode_thermal;
#endif

#if AP_QUICKTUNE_ENABLED
    AP_Quicktune quicktune;
#endif
    
    // 飞控系统状态（如 MANUAL、FBW-A、AUTO 等）
    Mode *control_mode = &mode_initializing;
    Mode *previous_mode = &mode_initializing;

    // 上一次模式切换时间
    uint32_t last_mode_change_ms;

    // 用于启用倒飞功能
    bool inverted_flight;

    // 上次执行横滚/俯仰稳定的时间
    uint32_t last_stabilize_ms;

    // 失效保护
    struct {
        // 跟踪通道 3（油门）是否低于失效保护阈值
        // RC 接收机在丢信号时应输出低油门
        bool rc_failsafe;

        // ADS-B 相关失效保护触发标记
        bool adsb;

        // 保存的飞行模式
        enum Mode::Number saved_mode_number;

        // 跟踪当前失效保护类型
        // 用于 RC 或 GCS 信号丢失的失效保护
        int16_t state;

        // 低油门次数计数
        uint8_t throttle_counter;

        uint32_t last_valid_rc_ms;

        // 记录与 AFS 相关的最后有效 RC 时间
        // 标准失效保护触发时不计为有效 RC 输入
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

#if HAL_QUADPLANE_ENABLED
    // 降落
    class VTOLApproach {
    public:
        enum class Stage {
            RTL,
            LOITER_TO_ALT,
            ENSURE_RADIUS,
            WAIT_FOR_BREAKOUT,
            APPROACH_LINE,
            VTOL_LANDING,
        };

        Stage approach_stage;
        float approach_direction_deg;
    } vtol_approach_s;
#endif

    bool any_failsafe_triggered() {
        return failsafe.state != FAILSAFE_NONE || battery.has_failsafed() || failsafe.adsb;
    }

    // 计数器：等待 GPS 解算稳定后再记录 Home（以及在空中启动时执行地面启动）
    uint8_t ground_start_count = 5;

    // 是否已有 AHRS 位置估计
    bool have_position;

    // 空速
    // FBW-B 使用的计算空速；也用于高阶模式保证最小地速
    // 亦用于襟翼放下判据，单位：cm/s
    int32_t target_airspeed_cm;
    int32_t new_airspeed_cm = -1;  // AUTO/GUIDED 模式变速的临时变量

    // 当前与目标空速差，用于俯仰控制器，单位：m/s
    float airspeed_error;

    // AUTO 模式下基于油门杆上半段增加空速的幅度，单位：cm/s
    int16_t airspeed_nudge_cm;

    // 类似 airspeed_nudge，但在无空速传感器时使用
    // 0-(throttle_max - throttle_cruise)：AUTO 模式下油门杆上半段的油门微调
    int16_t throttle_nudge;

    // 地速
    // 当前地速低于最小地速的幅度，单位：cm/s
    int32_t groundspeed_undershoot;
    bool groundspeed_undershoot_is_valid;
    float last_groundspeed_undershoot_offset;

    // 舵面速度缩放因子，10Hz 更新
    float surface_speed_scaler = 1.0;

    // 电池传感器
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t rudder_takeoff_warn_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
        uint32_t start_time_ms;
        bool waiting_for_rudder_neutral;
        float throttle_lim_max;
        float throttle_lim_min;
        uint32_t throttle_max_timer_ms;
        uint32_t level_off_start_time_ms;
        // 适合作为记录 TKOFF_THR_MAX_T 初始时间的候选
#if MODE_AUTOLAND_ENABLED
       struct {
            float heading; // 度
            bool initialized;
        } initial_direction;
#endif
    } takeoff_state;

    // 地面转向控制器状态
    struct {
        // 起飞与降落阶段保持的方向（百分之一度）
        // -1 表示航向未设置/未使用
        // 取值 0..36000，-1 表示禁用
        int32_t hold_course_cd = -1;

        // locked_course 与 locked_course_cd 用于自稳模式地面转向
        // 以及自动起飞时的转向
        bool locked_course;
        float locked_course_err;
        uint32_t last_steer_ms;
    } steer_state;

    // 飞行模式相关
    struct {
        // 自动模式中完成起飞指令的高度阈值（相对 Home，cm）
        int32_t takeoff_altitude_rel_cm;

        // 在该高度开始拉平起飞最小俯仰角，以减少/消除超调
        int32_t height_below_takeoff_to_level_off_cm;

        // 进入 AUTO 后达到的最大空速，用于控制地面起飞
        float highest_airspeed;

        // 任务下一航段的转弯角
        float next_turn_angle {90};

        // 降落时的滤波下沉率
        float sink_rate;

        // 距离下一个航点的距离
        float wp_distance;

        // 到下一航点的比例
        float wp_proportion;

        // is_flying() 最近一次返回 true 的时间（ms）
        uint32_t last_flying_ms;

        // AUTO 模式开始飞行的时间戳（ms）
        uint32_t started_flying_in_auto_ms;

        // 起飞时的气压高度
        float baro_takeoff_alt;

        // 初始俯仰角，用于检测尾轮机型机头抬起
        int16_t initial_pitch_cd;

        // 执行起飞指令时保持的最小俯仰角（0.01 度）
        int16_t takeoff_pitch_cd;

        // 使用 GPS 地航向替代 INS 航向的标志；起飞指令进行中置为 false
        bool takeoff_complete;

        // 是否正飞向降落进近航点（适用于任意导航类型）
        bool wp_is_land_approach;

        // 是否倒飞
        bool inverted_flight;

        // 下一航点是否启用横向偏差修正
        bool next_wp_crosstrack;

        // 当前航点是否使用横向偏差修正
        bool crosstrack;

        // 是否处于 FBWA 尾轮机起飞模式
        bool fbwa_tdrag_takeoff_mode;

        // 是否已检查自动降落
        bool checked_for_autoland;

        // 自动模式完成起飞指令的高度阈值（cm）
        // 是否处于 idle 模式：气球发射时用于在达到高度前禁止舵机动作
        bool idle_mode;
        
        // AUTO 模式下是否为 VTOL 模式
        bool vtol_mode;

        // 是否以 VTOL 方式盘旋
        bool vtol_loiter;

        // 地形数据补偿量
        float terrain_correction;

        // 用于检测变化的上次 Home 高度
        int32_t last_home_alt_cm;

        // 起飞转体是否完成（如适用）
        bool rotation_complete;
    } auto_state;

#if AP_SCRIPTING_ENABLED
    // 支持脚本化导航指令及验证
    struct {
        bool enabled;
        uint16_t id;
        float roll_rate_dps;
        float pitch_rate_dps;
        float yaw_rate_dps;
        float throttle_pct;
        uint32_t start_ms;
        uint32_t current_ms;
        float rudder_offset_pct;
        bool run_yaw_rate_controller;
    } nav_scripting;
#endif

    struct GuidedState {
        // 外部控制器指令的横滚/俯仰/偏航（0.01 度）
        Vector3l forced_rpy_cd;
        // 最近一次收到外部控制器指令的时间
        Vector3l last_forced_rpy_ms;

        // 外部控制器指令的油门百分比
        float forced_throttle;
        uint32_t last_forced_throttle_ms;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        // 空速调整
        float target_airspeed_cm = -1;  // 不默认 0，0 也是有效速度
        float target_airspeed_accel;
        uint32_t target_airspeed_time_ms;

        // 高度调整
        Location target_location;
        // target_location 的高度字段用于保存一些标志值
        bool target_location_alt_is_minus_one() const;

        float target_alt_rate;
        uint32_t target_alt_time_ms = 0;
        uint8_t target_mav_frame = -1;

        // 航向跟踪
        float target_heading = -4; // 不要默认 0 或 -1，这两者在弧度里都是有效航向
        float target_heading_accel_limit;
        uint32_t target_heading_time_ms;
        guided_heading_type_t target_heading_type;
        bool target_heading_limit;
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } guided_state;

#if AP_LANDINGGEAR_ENABLED
    // 起落架状态
    struct {
        AP_FixedWing::FlightStage last_flight_stage;
    } gear;
#endif

    struct {
        // 硬着陆后只在着陆后立即检查一次，避免搬动机体时触发坠机检测
        bool checkedHardLanding;

        // 坠机检测：为 true 表示已坠机
        bool is_crashed;

        // 撞击检测标志，通过 impact_timer_ms 在数秒后失效
        bool impact_detected;

        // 去抖计时器
        uint32_t debounce_timer_ms;

        // 去抖计数延迟时间
        uint32_t debounce_time_total_ms;

        // impact_detected 为 true 的持续时间，数秒后超时，用于限制 isFlyingProbability
        uint32_t impact_timer_ms;
    } crash_state;

    // 控制 AUTO 模式下的油门抑制
    bool throttle_suppressed;

#if AP_BATTERY_WATT_MAX_ENABLED
    // 降低油门以避免电池过流
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // 用于反推
    uint32_t throttle_watt_limit_timer_ms;
#endif

    AP_FixedWing::FlightStage flight_stage = AP_FixedWing::FlightStage::NORMAL;

    // 当前处于飞行状态的概率，范围 0..1，1 表示 100% 确认在飞行
    float isFlyingProbability;

    // is_flying() 的上一次值
    bool previous_is_flying;

    // 进入飞行状态以来的时间（ms）
    uint32_t started_flying_ms;

    // 地面模式：未解锁且未飞行时为 true
    bool ground_mode;

    // 导航控制变量
    // 即时期望的横滚角（0.01 度）
    int32_t nav_roll_cd;

    // 即时期望的俯仰角（0.01 度）
    int32_t nav_pitch_cd;

    // 气动载荷因子：在横滚限幅前由期望横滚计算，使用 1/cos(nav_roll)
    float aerodynamic_load_factor = 1.0f;

    // 平滑后的空速估计，用于限制横滚角
    float smoothed_airspeed;

    // 任务库
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};


#if HAL_PARACHUTE_ENABLED
    AP_Parachute parachute;
#endif

    // 地形处理
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain;
#endif

    AP_Landing landing{mission,ahrs,&TECS_controller,nav_controller,aparm,
            FUNCTOR_BIND_MEMBER(&Plane::set_target_altitude_proportion, void, const Location&, float),
            FUNCTOR_BIND_MEMBER(&Plane::constrain_target_altitude_location, void, const Location&, const Location&),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_relative_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::disarm_if_autoland_complete, void),
            FUNCTOR_BIND_MEMBER(&Plane::update_flight_stage, void)};
#if HAL_ADSB_ENABLED
    AP_ADSB adsb;
#endif  // HAL_ADSB_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
    // 避让启用 ADS-B 的载具（通常为有人机）
    AP_Avoidance_Plane avoidance_adsb{adsb};
#endif  // AP_ADSB_AVOIDANCE_ENABLED

    // Outback Challenge 失效保护支持
#if AP_ADVANCEDFAILSAFE_ENABLED
    AP_AdvancedFailsafe_Plane afs;
#endif

    /*
      meta data to support counting the number of circles in a loiter
    */
    struct {
        // 上一次目标方位角，用于更新 sum_cd
        int32_t old_target_bearing_cd;

        // 盘旋所需的总转角，用于 Loiter Turns 指令
        int32_t total_cd;

        // 盘旋已完成的总角度
        int32_t sum_cd;

        // 盘旋方向：1 为顺时针，-1 为逆时针
        int8_t direction;

        // 盘旋涉及高度时，该标志在至少到达一次高度后为 true
        bool reached_target_alt;

        // 检查上升气流导致无法持续下降盘旋的情形
        bool unable_to_achieve_target_alt;

        // 盘旋开始时间（ms）
        uint32_t start_time_ms;

        // 盘旋一圈开始时的高度，用于检测每圈高度变化
        // 仅在 sum_cd > 36000 时有效
        int32_t start_lap_alt_cm;
        int32_t next_sum_lap_cd;

        // Loiter Time 指令要求的盘旋时间（ms）
        uint32_t time_max_ms;

        // 控制器使用的当前盘旋半径（m）
        float radius;
    } loiter;

    // 条件指令
    // 条件指令使用的值（如延时、改高度等）
    // 例如变更高度指令中，该值为目标高度
    int32_t condition_value;

    // 用于检查条件指令状态的起始值
    // 例如延时指令中 condition_start 记录延时开始时间
    uint32_t condition_start;

    // 3D 位置向量
    // 位置结构在 AP_Common 中定义
    const Location &home = ahrs.get_home();

    // 上一个航点位置，用于轨迹跟随和高度坡度计算
    Location prev_WP_loc {};

    // 飞机当前位置
    Location current_loc {};

    // 当前/激活航点位置，用于高度坡度、轨迹跟随与盘旋计算
    Location next_WP_loc {};

    // 高度控制
    struct {
        // 目标海拔高度（cm），用于气压高度导航
        int32_t amsl_cm;

        // 上一航点与当前航点的高度差（cm），用于高度坡度处理
        int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
        // 是否正在进行地形跟随
        bool terrain_following;

        // 是否在等待加载地形数据以初始化地形跟随
        bool terrain_following_pending;

        // 目标地形相对高度（cm），terrain_following 为 true 时有效
        int32_t terrain_alt_cm;

        // 高度误差报告的前视量
        float lookahead;
#endif

        // FBWB/CRUISE 高度控制的上次输入
        float last_elevator_input;

        // 上次检查驾驶员高度控制的时间
        uint32_t last_elev_check_us;
    } target_altitude {};

    float relative_altitude;

    struct {
        uint32_t last_trim_check;
        uint32_t last_trim_save;
    } auto_trim;

    struct {
        bool done_climb;
    } rtl;

    // 未解锁时更新 Home 的上次时间
    uint32_t last_home_update_ms;

    // 相机/天线云台跟踪与稳定
#if HAL_MOUNT_ENABLED
    AP_Mount camera_mount;
#endif

    // 解锁/上锁管理类
    AP_Arming_Plane arming;

    AP_Param param_loader {var_info};

    // 外部控制库
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Plane external_control;
#endif

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];

#if HAL_QUADPLANE_ENABLED
    // QuadPlane 支持
    QuadPlane quadplane{ahrs};
#endif

#if AP_TUNING_ENABLED
    // 遥控器调参支持
    AP_Tuning_Plane tuning;
#endif

    static const struct LogStructure log_structure[];

    // 差动推力的方向舵混控增益（0-1）
    float rudder_dt;

    // 滑翔模式切换计时器
    uint32_t soaring_mode_timer_ms;

    // 非 AUTO 模式的地形禁用（由 RC 选项开关设置）
    bool non_auto_terrain_disable;
    bool terrain_disabled();
#if AP_TERRAIN_AVAILABLE
    bool terrain_enabled_in_current_mode() const;
    bool terrain_enabled_in_mode(Mode::Number num) const;
    enum class terrain_bitmask {
        ALL             = 1U << 0,
        FLY_BY_WIRE_B   = 1U << 1,
        CRUISE          = 1U << 2,
        AUTO            = 1U << 3,
        RTL             = 1U << 4,
        AVOID_ADSB      = 1U << 5,
        GUIDED          = 1U << 6,
        LOITER          = 1U << 7,
        CIRCLE          = 1U << 8,
        QRTL            = 1U << 9,
        QLAND           = 1U << 10,
        QLOITER         = 1U << 11,
        AUTOLAND        = 1U << 12,
    };
    struct TerrainLookupTable{
       Mode::Number mode_num;
       terrain_bitmask bitmask;
    };
    static const TerrainLookupTable Terrain_lookup[];
#endif

#if AP_QUICKTUNE_ENABLED
    void update_quicktune(void);
#endif

    // Attitude.cpp（姿态）
    void adjust_nav_pitch_throttle(void);
    void update_load_factor(void);
    void apply_load_factor_roll_limits(void);
    void adjust_altitude_target();
    void setup_alt_slope(void);
    int32_t get_RTL_altitude_cm() const;
    bool rangefinder_use(enum RangeFinderUse rangefinder_use) const;
    float relative_ground_altitude(enum RangeFinderUse rangefinder_use);
    float relative_ground_altitude(enum RangeFinderUse rangefinder_use, bool use_terrain_if_available);
    void set_target_altitude_current(void);
    void set_target_altitude_location(const Location &loc);
    int32_t relative_target_altitude_cm(void);
    void change_target_altitude(int32_t change_cm);
    void set_target_altitude_proportion(const Location &loc, float proportion);
#if AP_TERRAIN_AVAILABLE
    bool set_target_altitude_proportion_terrain(void);
#endif
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
    int32_t calc_altitude_error_cm(void);
    void check_fbwb_altitude(void);
    void reset_offset_altitude(void);
    void set_offset_altitude_location(const Location &start_loc, const Location &destination_loc);
    bool above_location_current(const Location &loc);
    void setup_terrain_target_alt(Location &loc);
    int32_t adjusted_altitude_cm(void);
    int32_t adjusted_relative_altitude_cm(void);
    float mission_alt_offset(void);
    float height_above_target(void);
    float lookahead_adjustment(void);
    void fix_terrain_WP(Location &loc, uint32_t linenum);
#if AP_RANGEFINDER_ENABLED
    float rangefinder_correction(void);
    void rangefinder_height_update(void);
    void rangefinder_terrain_correction(float &height);
#endif
    void stabilize();
    void calc_throttle();
    void calc_nav_roll();
    void calc_nav_pitch();
    float calc_speed_scaler(void);
    float get_speed_scaler(void) const { return surface_speed_scaler; }
    bool stick_mixing_enabled(void);
    void stabilize_roll();
    float stabilize_roll_get_roll_out();
    void stabilize_pitch();
    float stabilize_pitch_get_pitch_out();
    void stabilize_stick_mixing_fbw();
    void stabilize_yaw();
    int16_t calc_nav_yaw_coordinated();
    int16_t calc_nav_yaw_course(void);
    int16_t calc_nav_yaw_ground(void);

#if HAL_LOGGING_ENABLED

    // AP_Vehicle 接口方法
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;

    // Log.cpp（日志）
    void Log_Write_FullRate(void);
    void Log_Write_Attitude(void);
    void Log_Write_Control_Tuning();
    void Log_Write_OFG_Guided();
    void Log_Write_Guided(void);
    void Log_Write_Nav_Tuning();
    void Log_Write_Status();
    void Log_Write_RC(void);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Write_AETR();

#if AP_PLANE_BLACKBOX_LOGGING
    void Log_Write_Blackbox(void);
#endif
#endif

    // Parameters.cpp（参数）
    void load_parameters(void) override;

    // commands_logic.cpp（任务逻辑）
    void set_next_WP(const Location &loc);
    void do_RTL(int32_t alt);
    bool verify_takeoff();
    bool verify_loiter_unlim(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_time();
    bool verify_loiter_turns(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd);
    bool verify_continue_and_change_alt();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_altitude_wait(const AP_Mission::Mission_Command &cmd);
    void do_loiter_at_location();
    bool verify_loiter_heading(bool init);
    void exit_mission_callback();
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
#if HAL_QUADPLANE_ENABLED
    void do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
#endif
    void loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_loiter_turns(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);
    void do_altitude_wait(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
#if HAL_QUADPLANE_ENABLED
    // QuadPlane 的 VTOL 起飞（来自 AP_Vehicle）
    bool start_takeoff(const float alt_m) override;
    bool verify_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
#endif
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    bool do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    float get_wp_radius() const;

    bool is_land_command(uint16_t cmd) const;

    bool do_change_speed(SPEED_TYPE speedtype, float speed_target_ms, float rhtottle_pct);
    /*
      return true if in a specific AUTO mission command
    */
    bool in_auto_mission_id(uint16_t command) const;

#if AP_SCRIPTING_ENABLED
    // 导航脚本支持
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif

    // commands.cpp（任务命令）
    void set_guided_WP(const Location &loc);

    // 更新 Home 位置，成功更新返回 true
    bool update_home();

    // 更新 current_loc
    void update_current_loc(void);

    // 设置并持久化 Home 位置
    bool set_home_persistently(const Location &loc) WARN_IF_UNUSED;
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;

    // control_modes.cpp（模式控制）
    void autotune_start(void);
    void autotune_restore(void);
    void autotune_enable(bool enable);
    bool fly_inverted(void);
    uint8_t get_mode() const override { return (uint8_t)control_mode->mode_number(); }
    Mode *mode_from_mode_num(const enum Mode::Number num);
    bool current_mode_requires_mission() const override {
        return control_mode == &mode_auto;
    }

    bool autotuning;

    // events.cpp（事件）
    void rc_failsafe_short_on_event();
    void failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason);
    void rc_failsafe_short_off_event();
    void failsafe_long_off_event(ModeReason reason);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    bool failsafe_in_landing_sequence() const;  // 若处于降落序列则返回 true，仅用于失效保护代码

#if AP_FENCE_ENABLED
    // fence.cpp（围栏）
    void fence_check();
    void fence_checks_async() override;
    bool fence_stickmixing() const;
    bool in_fence_recovery() const;
    uint8_t orig_breaches;
#endif

    // Plane.cpp（主流程）
    void disarm_if_autoland_complete();
    bool trigger_land_abort(const float climb_to_alt_m);
    void get_osd_roll_pitch_rad(float &roll, float &pitch) const override;
    float tecs_hgt_afe(void);
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void ahrs_update();
    void update_speed_height(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_compass(void);
    void update_alt(void);
#if AP_ADVANCEDFAILSAFE_ENABLED
    void afs_fs_check(void);
#endif
    void one_second_loop(void);
    void three_hz_loop(void);
#if AP_AIRSPEED_AUTOCAL_ENABLE
    void airspeed_ratio_update(void);
#endif
    void update_logging10(void);
    void update_logging25(void);
    void update_control_mode(void);
    void update_fly_forward(void);
    void update_flight_stage();
    void set_flight_stage(AP_FixedWing::FlightStage fs);
    bool flight_option_enabled(FlightOptions flight_option) const;

    // navigation.cpp（导航）
    void loiter_angle_reset(void);
    void loiter_angle_update(void);
    void navigate();
    void check_home_alt_change(void);
    void calc_airspeed_errors();
    float mode_auto_target_airspeed_cm();
    void calc_gndspeed_undershoot();
    void update_loiter(uint16_t radius);
    void update_loiter_update_nav(uint16_t radius);
    void update_fbwb_speed_height(void);
    void setup_turn_angle(void);
    bool reached_loiter_target(void);

    // radio.cpp（遥控）
    void set_control_channels(void) override;
    void init_rc_in();
    void init_rc_out_main();
    void init_rc_out_aux();
    void read_radio();
    int16_t rudder_input(void);
    void control_failsafe();
    void trim_radio();
    bool rc_throttle_value_ok(void) const;
    bool rc_failsafe_active(void) const;

#if AP_RANGEFINDER_ENABLED
    // sensors.cpp（传感器）
    void read_rangefinder(void);
#endif

    // system.cpp（系统）
    __INITFUNC__ void init_ardupilot() override;
    bool set_mode(Mode& new_mode, const ModeReason reason);
    bool set_mode(const uint8_t mode, const ModeReason reason) override;
    bool set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason);
    void check_long_failsafe();
    void check_short_rc_failsafe();
    void startup_INS(void);
    bool should_log(uint32_t mask);
    int8_t throttle_percentage(void);
    void notify_mode(const Mode& mode);
    bool gcs_mode_enabled(const Mode::Number mode_num) const;

    // takeoff.cpp（起飞）
    bool auto_takeoff_check(void);
    void takeoff_calc_roll(void);
    void takeoff_calc_pitch(void);
    void takeoff_calc_throttle();
    int8_t takeoff_tail_hold(void);
    int16_t get_takeoff_pitch_min_cd(void);
    void landing_gear_update(void);
    bool check_takeoff_timeout(void);
    bool check_takeoff_timeout_level_off(void);

    // avoidance_adsb.cpp（ADSB 避障）
    void avoidance_adsb_update(void);

    // servos.cpp（舵机）
    void set_servos();
    float apply_throttle_limits(float throttle_in);
    void set_throttle(void);
    void set_takeoff_expected(void);
    void set_servos_flaps(void);
    void dspoiler_update(void);
    void airbrake_update(void);
    void landing_neutral_control_surface_servos(void);
    void servos_output(void);
    void servos_auto_trim(void);
    void servos_twin_engine_mix();
    void force_flare();
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);
    void throttle_slew_limit();
    bool suppress_throttle(void);
    void update_throttle_hover();
    void channel_function_mixer(SRV_Channel::Function func1_in, SRV_Channel::Function func2_in,
                                SRV_Channel::Function func1_out, SRV_Channel::Function func2_out) const;
    void flaperon_update();
    void indicate_waiting_for_rud_neutral_to_takeoff(void);

    // is_flying.cpp（飞行状态）
    void update_is_flying_5Hz(void);
    void crash_detection_update(void);
    bool in_preLaunch_flight_stage(void);
    bool is_flying(void);

    // parachute.cpp（降落伞）
    void parachute_check();
#if HAL_PARACHUTE_ENABLED
    void parachute_release();
    bool parachute_manual_release();
#endif

    // soaring.cpp（滑翔）
#if HAL_SOARING_ENABLED
    void update_soaring();
#endif

    // RC_Channel.cpp（RC 通道）
    bool emergency_landing;

    // 机型相关的航点信息辅助函数
    bool get_wp_distance_m(float &distance) const override;
    bool get_wp_bearing_deg(float &bearing) const override;
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;

    // reverse_thrust.cpp（反推）
    bool reversed_throttle;
    bool have_reverse_throttle_rc_option;
    bool allow_reverse_thrust(void) const;
    bool have_reverse_thrust(void) const;
    float get_throttle_input(bool no_deadzone=false) const;
    float get_adjusted_throttle_input(bool no_deadzone=false) const;
    bool reverse_thrust_enabled(UseReverseThrust use_reverse_thrust_option) const;

#if AP_SCRIPTING_ENABLED
    // 支持 NAV_SCRIPT_TIME 任务指令
    bool nav_scripting_active(void);
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;
    void nav_script_time_done(uint16_t id) override;

    // 下发油门百分比与横滚/俯仰/偏航角速度目标
    // 用于脚本控制器
    void set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps) override;
    void set_rudder_offset(float rudder_pct, bool run_yaw_rate_controller) override;
    bool nav_scripting_enable(uint8_t mode) override;
#endif
 
    enum Failsafe_Action {
        Failsafe_Action_None      = 0,
        Failsafe_Action_RTL       = 1,
        Failsafe_Action_Land      = 2,
        Failsafe_Action_Terminate = 3,
#if HAL_QUADPLANE_ENABLED
        Failsafe_Action_QLand     = 4,
#endif
        Failsafe_Action_Parachute = 5,
#if HAL_QUADPLANE_ENABLED
        Failsafe_Action_Loiter_alt_QLand = 6,
#endif
        Failsafe_Action_AUTOLAND_OR_RTL = 7,
    };

    // 优先级列表，优先级由高到低
    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Terminate,
                                                      Failsafe_Action_Parachute,
#if HAL_QUADPLANE_ENABLED
                                                      Failsafe_Action_QLand,
#endif
                                                      Failsafe_Action_Land,
                                                      Failsafe_Action_RTL,
                                                      Failsafe_Action_None,
                                                      -1 // 优先级列表必须以 -1 结束
                                                     };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // ekf_check.cpp 中的导航丢失 EKF 检查
    // 这些检查针对 VTOL 操作
    void ekf_check();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);

    enum class CrowMode {
        NORMAL,
        PROGRESSIVE,
        CROW_DISABLED,
    };

    using ThrFailsafe = Parameters::ThrFailsafe;

    CrowMode crow_mode = CrowMode::NORMAL;

    enum class FlareMode {
        FLARE_DISABLED = 0,
        ENABLED_NO_PITCH_TARGET,
        ENABLED_PITCH_TARGET
    };
    
    enum class AutoTuneAxis {
        ROLL  = 1U <<0,
        PITCH = 1U <<1,
        YAW   = 1U <<2,
    };

    FlareMode flare_mode;
    bool throttle_at_zero(void) const;

    // expo 处理
    float roll_in_expo(bool use_dz) const;
    float pitch_in_expo(bool use_dz) const;
    float rudder_in_expo(bool use_dz) const;

    // 进入上一个模式的原因
    ModeReason previous_mode_reason = ModeReason::UNKNOWN;

    // 最近传给 TECS 的目标高度
    int32_t tecs_target_alt_cm;

public:
    void failsafe_check(void);
    bool is_landing() const override;
    bool is_taking_off() const override;
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
    bool set_target_location(const Location& target_loc) override;
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if AP_SCRIPTING_ENABLED
    bool get_target_location(Location& target_loc) override;
    bool update_target_location(const Location &old_loc, const Location &new_loc) override;
    bool set_velocity_match(const Vector2f &velocity) override;

    // 允许脚本覆盖降落下沉率，允许为负以爬升
    bool set_land_descent_rate(float descent_rate) override;

    // 允许脚本覆盖任务/引导的横向偏差行为
    // 位置合理性由 Lua 脚本保证
    bool set_crosstrack_start(const Location &new_start_location) override;

#endif // AP_SCRIPTING_ENABLED

    bool tkoff_option_is_set(AP_FixedWing::TakeoffOption option) const {
        return (aparm.takeoff_options & int32_t(option)) != 0;
    }
   

};

extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
