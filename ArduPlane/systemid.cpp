#include "systemid.h"

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/control.h>
#include "Plane.h"

/*
  通过辅助开关控制系统辨识
 */

const AP_Param::GroupInfo AP_SystemID::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: 系统辨识轴
    // @Description: 控制被激励的轴，设为非零以显示更多参数
    // @User: Standard
    // @Values: 0:None, 1:VTOL Input Roll Angle, 2:VTOL Input Pitch Angle, 3:VTOL Input Yaw Angle, 4:VTOL Recovery Roll Angle, 5:VTOL Recovery Pitch Angle, 6:VTOL Recovery Yaw Angle, 7:VTOL Rate Roll, 8:VTOL Rate Pitch, 9:VTOL Rate Yaw, 10:VTOL Mixer Roll, 11:VTOL Mixer Pitch, 12:VTOL Mixer Yaw, 13:VTOL Mixer Thrust, 20:FW Input Roll Angle, 21:FW Input Pitch Angle, 22:FW Mixer Roll, 23:FW Mixer Pitch
    AP_GROUPINFO_FLAGS("_AXIS", 1, AP_SystemID, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: 系统辨识 Chirp 幅值
    // @Description: 扫频幅值，角度/角速度单位为度/度每秒，混控输出范围 0-1。
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, AP_SystemID, waveform_magnitude, 5),

    // @Param: _F_START_HZ
    // @DisplayName: 系统辨识起始频率
    // @Description: 扫频起始频率
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, AP_SystemID, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: 系统辨识终止频率
    // @Description: 扫频结束频率
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, AP_SystemID, frequency_stop, 15),

    // @Param: _T_FADE_IN
    // @DisplayName: 系统辨识淡入时间
    // @Description: 扫频达到最大幅值所需时间
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, AP_SystemID, time_fade_in, 5),

    // @Param: _T_REC
    // @DisplayName: 系统辨识扫频总时长
    // @Description: 完成扫频所需时间
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, AP_SystemID, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: 系统辨识淡出时间
    // @Description: 扫频结束后降至零幅值所需时间
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, AP_SystemID, time_fade_out, 1),

    // @Param: _XY_CTRL_MUL
    // @DisplayName: 系统辨识 XY 控制倍数
    // @Description: VTOL 模式下使用系统辨识时，对 XY 速度/位置控制器的倍率
    // @Range: 0.05 1.0
    // @User: Standard
    AP_GROUPINFO("_XY_CTRL_MUL", 8, AP_SystemID, xy_control_mul, 0.1),
    
    AP_GROUPEND
};

AP_SystemID::AP_SystemID(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// @LoggerMessage: SIDS
// @Description: 系统辨识设置
// @Field: TimeUS: 系统启动以来的时间
// @Field: Ax: 被激励的轴
// @Field: Mag: Chirp 波形幅值
// @Field: FSt: Chirp 起始频率
// @Field: FSp: Chirp 结束频率
// @Field: TFin: Chirp 达到最大幅值所需时间
// @Field: TC: Chirp 开始前的恒定频率时间
// @Field: TR: 完成 Chirp 波形所需时间
// @Field: TFout: Chirp 结束后降至零幅值所需时间

/*
  启动系统辨识
 */
void AP_SystemID::start()
{
    start_axis = axis;

    switch (start_axis) {
        case AxisType::NONE:
            // 检查是否启用
            gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: No axis selected");
            return;
        case AxisType::INPUT_ROLL:
        case AxisType::INPUT_PITCH:
        case AxisType::INPUT_YAW:
        case AxisType::RECOVER_ROLL:
        case AxisType::RECOVER_PITCH:
        case AxisType::RECOVER_YAW:
        case AxisType::RATE_ROLL:
        case AxisType::RATE_PITCH:
        case AxisType::RATE_YAW:
        case AxisType::MIX_ROLL:
        case AxisType::MIX_PITCH:
        case AxisType::MIX_YAW:
        case AxisType::MIX_THROTTLE:
            // 当前飞行模式或阶段不支持系统辨识轴则退出
            if (!plane.control_mode->supports_vtol_systemid()) {
#if HAL_QUADPLANE_ENABLED
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported for this flight mode");
#else
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported in Plane");
#endif
                return;
            }
            break;
        case AxisType::FW_INPUT_ROLL:
        case AxisType::FW_INPUT_PITCH:
        case AxisType::FW_MIX_ROLL:
        case AxisType::FW_MIX_PITCH:
            // 当前飞行模式或阶段不支持系统辨识轴则退出
            if (!plane.control_mode->supports_fw_systemid()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported for this flight mode");
                return;
            }
            if (!plane.control_mode->allow_fw_systemid()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported for this flight phase");
                return;
            }
            break;
    }

    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: must be armed");
        return;
    }

    attitude_offset_deg.zero();
    throttle_offset = 0;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        restore.att_bf_feedforward = plane.quadplane.attitude_control->get_bf_feedforward();
    }
#endif

    waveform_time = 0;
    time_const_freq = 2.0 / frequency_start; // 起始频率下的两个完整周期

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("SIDS", "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout",
                                "s--ssssss", "F--------", "QBfffffff",
                                AP_HAL::micros64(),
                                uint8_t(start_axis),
                                waveform_magnitude.get(),
                                frequency_start.get(),
                                frequency_stop.get(),
                                time_fade_in.get(),
                                time_const_freq,
                                time_record.get(),
                                time_fade_out.get());
#endif // HAL_LOGGING_ENABLED

    running = true;
}

/*
  停止系统辨识
 */
void AP_SystemID::stop()
{
    if (running) {
        running = false;
        attitude_offset_deg.zero();
        throttle_offset = 0;

#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.available()) {
            auto *attitude_control = plane.quadplane.attitude_control;
            attitude_control->bf_feedforward(restore.att_bf_feedforward);
            attitude_control->rate_bf_roll_sysid_rads(0);
            attitude_control->rate_bf_pitch_sysid_rads(0);
            attitude_control->rate_bf_yaw_sysid_rads(0);
            plane.quadplane.pos_control->NE_set_control_scale_factor(1);

            // 重新初始化 XY 控制器，使当前位置作为目标
            plane.quadplane.pos_control->NE_init_controller();
        }
#endif
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID stopped");
    }
}

/*
  更新系统辨识（需要主循环频率调用）
 */
void AP_SystemID::vtol_update()
{
#if HAL_QUADPLANE_ENABLED
    if (!running) {
        return;
    }
    if (chirp_input.completed()) {
        stop();
        return;
    }

    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();

    waveform_time += last_loop_time_s;
    waveform_sample = chirp_input.update(waveform_time, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    auto *attitude_control = plane.quadplane.attitude_control;

    switch (start_axis) {
        case AxisType::NONE:
            // 不可能发生，见 start()
            break;
        case AxisType::INPUT_ROLL:
            attitude_offset_deg.x = waveform_sample;
            break;
        case AxisType::INPUT_PITCH:
            attitude_offset_deg.y = waveform_sample;
            break;
        case AxisType::INPUT_YAW:
            attitude_offset_deg.z = waveform_sample;
            break;
        case AxisType::RECOVER_ROLL:
            attitude_offset_deg.x = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_PITCH:
            attitude_offset_deg.y = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_YAW:
            attitude_offset_deg.z = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RATE_ROLL:
            attitude_control->rate_bf_roll_sysid_rads(radians(waveform_sample));
            break;
        case AxisType::RATE_PITCH:
            attitude_control->rate_bf_pitch_sysid_rads(radians(waveform_sample));
            break;
        case AxisType::RATE_YAW:
            attitude_control->rate_bf_yaw_sysid_rads(radians(waveform_sample));
            break;
        case AxisType::MIX_ROLL:
            attitude_control->actuator_roll_sysid(waveform_sample);
            break;
        case AxisType::MIX_PITCH:
            attitude_control->actuator_pitch_sysid(waveform_sample);
            break;
        case AxisType::MIX_YAW:
            attitude_control->actuator_yaw_sysid(waveform_sample);
            break;
        case AxisType::MIX_THROTTLE:
            throttle_offset = waveform_sample;
            break;
        default:
            break;
    }

    // 在位置控制模式下降低 NE 轴控制量
    plane.quadplane.pos_control->NE_set_control_scale_factor(xy_control_mul);

    if (log_subsample <= 0) {
        log_data();
        // 同频记录姿态控制器
        plane.quadplane.Log_Write_AttRate();

        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;

#endif
}

// 当前是否在运行固定翼系统辨识
bool AP_SystemID::is_running_fw() const
{
    if (!is_running()) {
        return false;
    }

    switch (start_axis) {
        case AxisType::NONE:
        case AxisType::INPUT_ROLL:
        case AxisType::INPUT_PITCH:
        case AxisType::INPUT_YAW:
        case AxisType::RECOVER_ROLL:
        case AxisType::RECOVER_PITCH:
        case AxisType::RECOVER_YAW:
        case AxisType::RATE_ROLL:
        case AxisType::RATE_PITCH:
        case AxisType::RATE_YAW:
        case AxisType::MIX_ROLL:
        case AxisType::MIX_PITCH:
        case AxisType::MIX_YAW:
        case AxisType::MIX_THROTTLE:
            break;

        case AxisType::FW_INPUT_ROLL:
        case AxisType::FW_INPUT_PITCH:
        case AxisType::FW_MIX_ROLL:
        case AxisType::FW_MIX_PITCH:
            return true;
    }

    return false;
}

/*
  更新系统辨识（需要主循环频率调用）
 */
void AP_SystemID::fw_update()
{
    if (!plane.control_mode->allow_fw_systemid() || chirp_input.completed()) {
        // 控制模式变化意味着需要停止 chirp，或 chirp 已完成
        stop();
        return;
    }

    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();

    waveform_time += last_loop_time_s;
    waveform_sample = chirp_input.update(waveform_time, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    switch (start_axis) {
        case AxisType::NONE:
            // 不可能发生，见 start()
            break;
        case AxisType::FW_INPUT_ROLL:
            plane.nav_roll_cd += waveform_sample * 100.0f;
            break;
        case AxisType::FW_INPUT_PITCH:
            plane.nav_pitch_cd += waveform_sample * 100.0f;
            break;
        case AxisType::FW_MIX_ROLL:
            output_offset.x = waveform_sample;
            break;
        case AxisType::FW_MIX_PITCH:
            output_offset.y = waveform_sample;
            break;
        default:
            break;
    }

    if (log_subsample <= 0) {
        log_data();
        log_plane_data();

        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// @LoggerMessage: SIDD
// @Description: 系统辨识数据
// @Field: TimeUS: 系统启动以来的时间
// @Field: Time: 波形时间参考
// @Field: Targ: 当前波形采样值
// @Field: F: 瞬时波形频率
// @Field: Gx: 角度增量，X 轴
// @Field: Gy: 角度增量，Y 轴
// @Field: Gz: 角度增量，Z 轴
// @Field: Ax: 速度增量，X 轴
// @Field: Ay: 速度增量，Y 轴
// @Field: Az: 速度增量，Z 轴

// 记录系统辨识
void AP_SystemID::log_data() const
{
#if HAL_LOGGING_ENABLED
    Vector3f delta_angle;
    float delta_angle_dt;
    plane.ins.get_delta_angle(delta_angle, delta_angle_dt);

    Vector3f delta_velocity;
    float delta_velocity_dt;
    plane.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        const float dt_ang_inv = 1.0 / delta_angle_dt;
        const float dt_vel_inv = 1.0 / delta_velocity_dt;
        AP::logger().WriteStreaming("SIDD", "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az",
                                    "ss-zkkkooo", "F---------", "Qfffffffff",
                                    AP_HAL::micros64(),
                                    waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI),
                                    degrees(delta_angle.x * dt_ang_inv),
                                    degrees(delta_angle.y * dt_ang_inv),
                                    degrees(delta_angle.z * dt_ang_inv),
                                    delta_velocity.x * dt_vel_inv,
                                    delta_velocity.y * dt_vel_inv,
                                    delta_velocity.z * dt_vel_inv);
    }
#endif // HAL_LOGGING_ENABLED
}

// @LoggerMessage: SIDP
// @Description: 固定翼系统辨识数据
// @Field: TimeUS: 系统启动以来的时间
// @Field: DRll: 期望横滚角
// @Field: Rll: 横滚角
// @Field: DPit: 期望俯仰角
// @Field: Pit: 俯仰角
// @Field: rdes: 期望横滚角速度
// @Field: r: 实测横滚角速度
// @Field: pdes: 期望俯仰角速度
// @Field: p: 实测俯仰角速度
// @Field: Aile: 副翼
// @Field: Elev: 升降舵
// @Field: aspd: 速度缩放
// @Field: eastas: EAS2TAS

void AP_SystemID::log_plane_data() const
{
#if HAL_LOGGING_ENABLED
   // int16_t pitch = plane.ahrs.pitch_sensor - plane.g.pitch_trim * 100;
    float speed_scaler = plane.get_speed_scaler();
    const auto &pitch_pid_info = plane.pitchController.get_pid_info();
    const auto &roll_pid_info = plane.rollController.get_pid_info();

    int16_t demanded_pitch = plane.nav_pitch_cd + int32_t(plane.g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * plane.g.kff_throttle_to_pitch;
    AP::logger().WriteStreaming("SIDP", "TimeUS,DRll,Rll,DPit,Pit,rdes,r,pdes,p,Aile,Elev,aspd,eastas",
                                "soooooooooooo", "F------------", "Qffffffffffff",
                                AP_HAL::micros64(),
                                plane.nav_roll_cd * 0.01f,
                                plane.ahrs.roll_sensor * 0.01f,
                                demanded_pitch * 0.01f,
                                plane.ahrs.pitch_sensor * 0.01f,
                                roll_pid_info.target,
                                degrees(plane.ahrs.get_gyro().x),
                                pitch_pid_info.target,
                                degrees(plane.ahrs.get_gyro().y),
                                SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * 0.01f,
                                SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * 0.01f,
                                speed_scaler,
                                plane.ahrs.get_EAS2TAS());

#endif // HAL_LOGGING_ENABLED
}
#endif // AP_PLANE_SYSTEMID_ENABLED

