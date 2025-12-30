## 项目目录结构

以下为项目根目录的一级目录与同级文件（说明直接跟在条目后）：

```text
.
├─ .git — Git 版本库与元数据
├─ .github — GitHub 工作流与配置
├─ .idea — JetBrains 系列 IDE 项目配置
├─ .vscode — VS Code 项目配置
├─ AntennaTracker — 天线跟踪器相关代码
├─ ArduCopter — 多旋翼飞行器固件
├─ ArduPlane — 固定翼飞行器固件
├─ ArduSub — 水下航行器固件
├─ benchmarks — 性能基准与测试用例
├─ Blimp — 飞艇相关固件
├─ docs — 项目文档
├─ libraries — 通用库与共享组件
├─ modules — 子模块集合
├─ Rover — 地面车/无人车固件
├─ tests — 测试代码与测试资源
├─ Tools — 构建、脚本与辅助工具
├─ .dir-locals.el — 编辑器局部设置（Emacs）
├─ .dockerignore — Docker 构建忽略规则
├─ .editorconfig — 代码风格配置
├─ .flake8 — Python 代码检查配置
├─ .git-blame-ignore-revs — Git blame 忽略提交列表
├─ .gitattributes — Git 属性配置
├─ .gitignore — Git 忽略规则
├─ .gitmodules — Git 子模块配置
├─ .pre-commit-config.yaml — pre-commit 钩子配置
├─ .valgrind-suppressions — Valgrind 抑制规则
├─ .valgrindrc — Valgrind 默认参数
├─ BUILD.md — 构建说明
├─ CODE_OF_CONDUCT.md — 行为准则
├─ COPYING.txt — 版权与许可证
├─ Dockerfile — Docker 构建文件
├─ Doxyfile.in — Doxygen 配置模板
├─ Makefile — 构建入口
├─ pyproject.toml — Python 项目配置
├─ README.md — 项目说明（英文）
├─ READMEzh.md — 项目说明（中文）
├─ Vagrantfile — Vagrant 环境配置
├─ waf — Waf 构建脚本
└─ wscript — Waf 构建描述
```

ArduPlane 目录文件说明（主要关注 `ArduPlane/` 下的文件）：

- `ArduPlane/Plane.h` — 固定翼主类声明。
- `ArduPlane/Plane.cpp` — 固定翼主类实现与主流程。
- `ArduPlane/Parameters.h` — 固定翼参数声明与分组。
- `ArduPlane/Parameters.cpp` — 固定翼参数定义与默认值。
- `ArduPlane/defines.h` — 固定翼相关编译宏与常量定义。
- `ArduPlane/config.h` — 固定翼编译配置开关。
- `ArduPlane/version.h` — 版本与编译信息。
- `ArduPlane/Log.cpp` — 固定翼日志项定义与记录。
- `ArduPlane/system.cpp` — 系统级初始化与运行维护。
- `ArduPlane/systemid.h` — 系统标识相关声明。
- `ArduPlane/systemid.cpp` — 系统标识相关实现。
- `ArduPlane/commands.cpp` — 任务指令处理入口。
- `ArduPlane/commands_logic.cpp` — 任务指令逻辑实现。
- `ArduPlane/navigation.cpp` — 导航与航迹控制逻辑。
- `ArduPlane/altitude.cpp` — 高度控制与管理。
- `ArduPlane/Attitude.cpp` — 姿态控制相关逻辑。
- `ArduPlane/control_modes.cpp` — 模式切换与模式映射。
- `ArduPlane/mode.h` — 飞行模式基类与接口。
- `ArduPlane/mode.cpp` — 飞行模式基类实现。
- `ArduPlane/mode_manual.cpp` — 手动模式。
- `ArduPlane/mode_stabilize.cpp` — 自稳模式。
- `ArduPlane/mode_acro.cpp` — 特技模式。
- `ArduPlane/mode_fbwa.cpp` — FBWA 模式。
- `ArduPlane/mode_fbwb.cpp` — FBWB 模式。
- `ArduPlane/mode_cruise.cpp` — 巡航模式。
- `ArduPlane/mode_circle.cpp` — 盘旋模式。
- `ArduPlane/mode_training.cpp` — 训练模式。
- `ArduPlane/mode_autotune.cpp` — 自动调参模式。
- `ArduPlane/mode_auto.cpp` — 自动任务模式。
- `ArduPlane/mode_rtl.cpp` — 返航模式。
- `ArduPlane/mode_loiter.cpp` — 定点/盘旋等待模式。
- `ArduPlane/mode_guided.cpp` — 引导模式。
- `ArduPlane/mode_takeoff.cpp` — 起飞模式。
- `ArduPlane/mode_autoland.cpp` — 自动降落模式。
- `ArduPlane/mode_avoidADSB.cpp` — ADS-B 避障模式。
- `ArduPlane/mode_thermal.cpp` — 热气流滑翔模式。
- `ArduPlane/mode_LoiterAltQLand.cpp` — LoiterAltQLand 混合模式。
- `ArduPlane/mode_qacro.cpp` — QACRO 模式（垂起/四旋）。
- `ArduPlane/mode_qautotune.cpp` — QAUTOTUNE 模式（垂起/四旋）。
- `ArduPlane/mode_qhover.cpp` — QHOVER 模式（垂起/四旋）。
- `ArduPlane/mode_qland.cpp` — QLAND 模式（垂起/四旋）。
- `ArduPlane/mode_qloiter.cpp` — QLOITER 模式（垂起/四旋）。
- `ArduPlane/mode_qrtl.cpp` — QRTL 模式（垂起/四旋）。
- `ArduPlane/mode_qstabilize.cpp` — QSTABILIZE 模式（垂起/四旋）。
- `ArduPlane/quadplane.h` — QuadPlane 模块声明。
- `ArduPlane/quadplane.cpp` — QuadPlane 模块实现。
- `ArduPlane/transition.h` — 固定翼/垂起过渡相关声明。
- `ArduPlane/VTOL_Assist.h` — VTOL 辅助相关声明。
- `ArduPlane/VTOL_Assist.cpp` — VTOL 辅助相关实现。
- `ArduPlane/tailsitter.h` — 尾座式机型声明。
- `ArduPlane/tailsitter.cpp` — 尾座式机型实现。
- `ArduPlane/tiltrotor.h` — 倾转旋翼机型声明。
- `ArduPlane/tiltrotor.cpp` — 倾转旋翼机型实现。
- `ArduPlane/qautotune.h` — QuadPlane 自动调参声明。
- `ArduPlane/qautotune.cpp` — QuadPlane 自动调参实现。
- `ArduPlane/tuning.h` — 调参辅助声明。
- `ArduPlane/tuning.cpp` — 调参辅助实现。
- `ArduPlane/takeoff.cpp` — 起飞流程与状态机。
- `ArduPlane/pullup.h` — 拉起保护声明。
- `ArduPlane/pullup.cpp` — 拉起保护实现。
- `ArduPlane/parachute.cpp` — 降落伞相关逻辑。
- `ArduPlane/reverse_thrust.cpp` — 反推控制逻辑。
- `ArduPlane/soaring.cpp` — 滑翔与升力控制逻辑。
- `ArduPlane/servos.cpp` — 舵面/电机输出与混控。
- `ArduPlane/sensors.cpp` — 传感器管理与融合接口。
- `ArduPlane/is_flying.cpp` — 飞行状态判定。
- `ArduPlane/ekf_check.cpp` — EKF 健康检查。
- `ArduPlane/failsafe.cpp` — 失效保护处理。
- `ArduPlane/fence.cpp` — 围栏与地理限制。
- `ArduPlane/events.cpp` — 事件处理与通知。
- `ArduPlane/radio.cpp` — 遥控输入相关逻辑。
- `ArduPlane/RC_Channel_Plane.h` — 固定翼 RC 通道声明。
- `ArduPlane/RC_Channel_Plane.cpp` — 固定翼 RC 通道实现。
- `ArduPlane/GCS_Plane.h` — 固定翼 GCS 扩展声明。
- `ArduPlane/GCS_Plane.cpp` — 固定翼 GCS 扩展实现。
- `ArduPlane/GCS_MAVLink_Plane.h` — 固定翼 MAVLink 扩展声明。
- `ArduPlane/GCS_MAVLink_Plane.cpp` — 固定翼 MAVLink 扩展实现。
- `ArduPlane/avoidance_adsb.h` — ADS-B 避障声明。
- `ArduPlane/avoidance_adsb.cpp` — ADS-B 避障实现。
- `ArduPlane/AP_ExternalControl_Plane.h` — 外部控制接口声明。
- `ArduPlane/AP_ExternalControl_Plane.cpp` — 外部控制接口实现。
- `ArduPlane/AP_Arming_Plane.h` — 解锁检查声明。
- `ArduPlane/AP_Arming_Plane.cpp` — 解锁检查实现。
- `ArduPlane/afs_plane.h` — 高级失效保护（AFS）声明。
- `ArduPlane/afs_plane.cpp` — 高级失效保护（AFS）实现。
- `ArduPlane/motor_test.cpp` — 电机测试逻辑。
- `ArduPlane/ReleaseNotes.txt` — 固定翼发布说明。
- `ArduPlane/Makefile.waf` — Waf 构建规则。
- `ArduPlane/wscript` — Waf 构建脚本入口。
- `ArduPlane/createTags` — tags 生成脚本。
