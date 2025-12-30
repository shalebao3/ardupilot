#pragma once

#include "quadplane.h"

#ifndef AP_PLANE_SYSTEMID_ENABLED
// 通过自定义构建服务器启用
#define AP_PLANE_SYSTEMID_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_SITL && HAL_QUADPLANE_ENABLED
#endif

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/chirp.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/vector3.h>
#include <AP_Math/vector2.h>

class AP_SystemID {

public:
    AP_SystemID(void);
    void start(void);
    void stop(void);
    void fw_update();
    void vtol_update();

    static const struct AP_Param::GroupInfo var_info[];

    const Vector3f &get_attitude_offset_deg(void) const {
        return attitude_offset_deg;
    }
    const Vector3f &get_output_offset(void) const {
        return output_offset;
    }
    float get_throttle_offset(void) const {
        return running ? throttle_offset : 0.0;
    }

    bool is_running() const {
        return running;
    }

    // 当前是否在运行固定翼系统辨识
    bool is_running_fw() const;

private:
    Chirp chirp_input;
    bool running;
    // AxisType 14-19 不支持固定翼或 quadplane
    enum class AxisType {
        NONE = 0,               // 无
        INPUT_ROLL = 1,         // 激励角度输入横滚轴
        INPUT_PITCH = 2,        // 激励角度输入俯仰轴
        INPUT_YAW = 3,          // 激励角度输入偏航轴
        RECOVER_ROLL = 4,       // 激励角度恢复横滚轴
        RECOVER_PITCH = 5,      // 激励角度恢复俯仰轴
        RECOVER_YAW = 6,        // 激励角度恢复偏航轴
        RATE_ROLL = 7,          // 激励角速度横滚轴
        RATE_PITCH = 8,         // 激励角速度俯仰轴
        RATE_YAW = 9,           // 激励角速度偏航轴
        MIX_ROLL = 10,          // 激励混控横滚轴
        MIX_PITCH = 11,         // 激励混控俯仰轴
        MIX_YAW = 12,           // 激励混控偏航轴
        MIX_THROTTLE = 13,      // 激励混控油门轴
        FW_INPUT_ROLL = 20,     // 固定翼角度输入横滚轴激励
        FW_INPUT_PITCH = 21,    // 固定翼角度输入俯仰轴激励
        FW_MIX_ROLL = 22,       // 固定翼混控横滚轴激励
        FW_MIX_PITCH = 23,      // 固定翼混控俯仰轴激励
    };

    void set_bf_feedforward(bool value);
    void log_data() const;
    void log_plane_data() const;
    int8_t log_subsample;       // 日志子采样倍数


    AP_Enum<AxisType> axis;               // 控制被激励的轴，非零则显示其它参数
    AP_Float waveform_magnitude;// Chirp 波形幅值
    AP_Float frequency_start;   // Chirp 起始频率
    AP_Float frequency_stop;    // Chirp 结束频率
    AP_Float time_fade_in;      // 到达最大幅值的时间
    AP_Float time_record;       // 完成 Chirp 波形的时间
    AP_Float time_fade_out;     // Chirp 结束后衰减到零的时间
    AP_Float xy_control_mul;    // VTOL XY 控制倍数

    struct {
        bool att_bf_feedforward;    // attitude_control->get_bf_feedforward 的设置
    } restore;

    float waveform_time;        // 波形时间参考
    float waveform_sample;      // 当前波形采样值
    float waveform_freq_rads;   // 瞬时波形频率
    float time_const_freq;      // Chirp 开始前的恒定频率时间
    uint32_t last_loop_time_ms;   // 上次循环时间（ms）

    // 当前姿态偏移
    Vector3f attitude_offset_deg;
    Vector3f output_offset;
    float throttle_offset;

    AxisType start_axis;

};

#endif // AP_PLANE_SYSTEMID_ENABLED
