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

ArduPlane 目录文件说明（主要关注 `ArduPlane/` 下的文件，按功能分组）：

核心与配置：
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

任务、导航与控制：
- `ArduPlane/commands.cpp` — 任务指令处理入口。
- `ArduPlane/commands_logic.cpp` — 任务指令逻辑实现。
- `ArduPlane/navigation.cpp` — 导航与航迹控制逻辑。
- `ArduPlane/altitude.cpp` — 高度控制与管理。
- `ArduPlane/Attitude.cpp` — 姿态控制相关逻辑。
- `ArduPlane/control_modes.cpp` — 模式切换与模式映射。
- `ArduPlane/tuning.h` — 调参辅助声明。
- `ArduPlane/tuning.cpp` — 调参辅助实现。
- `ArduPlane/takeoff.cpp` — 起飞流程与状态机。
- `ArduPlane/pullup.h` — 拉起保护声明。
- `ArduPlane/pullup.cpp` — 拉起保护实现。
- `ArduPlane/soaring.cpp` — 滑翔与升力控制逻辑。
- `ArduPlane/is_flying.cpp` — 飞行状态判定。
- `ArduPlane/ekf_check.cpp` — EKF 健康检查。

固定翼飞行模式：
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

VTOL 与 QuadPlane：
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
- `ArduPlane/mode_qacro.cpp` — QACRO 模式（垂起/四旋）。
- `ArduPlane/mode_qautotune.cpp` — QAUTOTUNE 模式（垂起/四旋）。
- `ArduPlane/mode_qhover.cpp` — QHOVER 模式（垂起/四旋）。
- `ArduPlane/mode_qland.cpp` — QLAND 模式（垂起/四旋）。
- `ArduPlane/mode_qloiter.cpp` — QLOITER 模式（垂起/四旋）。
- `ArduPlane/mode_qrtl.cpp` — QRTL 模式（垂起/四旋）。
- `ArduPlane/mode_qstabilize.cpp` — QSTABILIZE 模式（垂起/四旋）。

传感器与输出：
- `ArduPlane/sensors.cpp` — 传感器管理与融合接口。
- `ArduPlane/servos.cpp` — 舵面/电机输出与混控。
- `ArduPlane/reverse_thrust.cpp` — 反推控制逻辑。
- `ArduPlane/motor_test.cpp` — 电机测试逻辑。

安全与失效保护：
- `ArduPlane/failsafe.cpp` — 失效保护处理。
- `ArduPlane/fence.cpp` — 围栏与地理限制。
- `ArduPlane/events.cpp` — 事件处理与通知。
- `ArduPlane/avoidance_adsb.h` — ADS-B 避障声明。
- `ArduPlane/avoidance_adsb.cpp` — ADS-B 避障实现。
- `ArduPlane/parachute.cpp` — 降落伞相关逻辑。
- `ArduPlane/AP_Arming_Plane.h` — 解锁检查声明。
- `ArduPlane/AP_Arming_Plane.cpp` — 解锁检查实现。
- `ArduPlane/afs_plane.h` — 高级失效保护（AFS）声明。
- `ArduPlane/afs_plane.cpp` — 高级失效保护（AFS）实现。

通信与外部控制：
- `ArduPlane/radio.cpp` — 遥控输入相关逻辑。
- `ArduPlane/RC_Channel_Plane.h` — 固定翼 RC 通道声明。
- `ArduPlane/RC_Channel_Plane.cpp` — 固定翼 RC 通道实现。
- `ArduPlane/GCS_Plane.h` — 固定翼 GCS 扩展声明。
- `ArduPlane/GCS_Plane.cpp` — 固定翼 GCS 扩展实现。
- `ArduPlane/GCS_MAVLink_Plane.h` — 固定翼 MAVLink 扩展声明。
- `ArduPlane/GCS_MAVLink_Plane.cpp` — 固定翼 MAVLink 扩展实现。
- `ArduPlane/AP_ExternalControl_Plane.h` — 外部控制接口声明。
- `ArduPlane/AP_ExternalControl_Plane.cpp` — 外部控制接口实现。

构建与辅助：
- `ArduPlane/ReleaseNotes.txt` — 固定翼发布说明。
- `ArduPlane/Makefile.waf` — Waf 构建规则。
- `ArduPlane/wscript` — Waf 构建脚本入口。
- `ArduPlane/createTags` — tags 生成脚本。

1) 先纠偏：ArduPlane 更像“调度器 + 状态机”，不是“消息队列”

你把 PX4 理解成“uORB 数据总线 + pub/sub”没问题；但 ArduPilot/ArduPlane 的内核更像：

一个主循环（loop）

一个高频调度器（AP_Scheduler）把任务按频率/优先级跑起来

一个“当前飞行模式 control_mode”（策略/状态机）在每个循环里更新

模块之间更多是共享对象/共享状态，而不是 uORB 那种强约束的数据总线

并且别被 setup()/loop() 迷惑：ArduPilot 并不等同于单线程玩到底，文档明确提到它“看起来像单线程但实际上不是”（有多线程/中断/IO 等并发来源）。 
ArduPilot.org

2) ArduPlane 默认代码调用链（从“入口”到“飞控核心循环”）

按你习惯的“从 main 往下读”，ArduPlane 的入口非常干净：

A. 程序入口：ArduPlane/ArduPlane.cpp

你会看到这一句（相当于“把 SpringBoot 应用启动起来”）：

AP_HAL_MAIN_CALLBACKS(&plane); 
ArduPilot Firmware Download
+1

B. 展开宏：libraries/AP_HAL/AP_HAL_Main.h

AP_HAL_MAIN_CALLBACKS 会生成真正的 main()，并把一个“包含 setup()/loop() 的对象”交给 HAL 去跑（可以理解为：把你的 Application 交给容器托管）。 
GitHub
+1

C. 车辆抽象：libraries/AP_Vehicle/AP_Vehicle.h

这里直接解释了：setup() 启动初始化一次，loop() 会被持续调用；AP_HAL_MAIN_CALLBACKS 引用的就是包含这两个函数的对象。 
ArduPilot Firmware Download
+1

3) “主循环里到底干嘛”：调度表就是你的“调用链地图”

在 ArduPlane 里，你真正该盯住的不是 while(true)，而是 scheduler 表。
这就像你在 SpringBoot 里要盯住：Filter/Interceptor/Controller，而不是盯 Tomcat 的 accept()。

核心：ArduPlane/Plane.cpp → Plane::scheduler_tasks[]

这里定义了所有周期任务（频率/优先级），而且最关键的 4 个是 FAST_TASK，每次 loop 都会跑：

ahrs_update

update_control_mode

stabilize

set_servos 
ArduPilot Firmware Download

这四个，几乎就是“从感知到执行”的最短闭环。

4) 飞行模式（Mode）是“策略实现”，切换模式像“路由切换”
A. 每次循环都会执行当前模式的更新

update_control_mode() 的核心一句非常关键：

control_mode->update(); 
ArduPilot Firmware Download
+1

这句话在架构上等价于：当前策略对象处理一次 tick。
类比 Spring：DispatcherServlet 根据路由把请求交给不同 Controller；这里是根据模式把“本周期控制逻辑”交给不同 Mode。

B. 模式是怎么切的：ArduPlane/system.cpp → Plane::set_mode(...)

你会看到非常典型的“状态机切换模板”：

如果目标模式就是当前模式，直接返回

校验（例如 VTOL 模式前置条件、围栏恢复时禁止切等）

先把 control_mode 指针切过去

调 new_mode.enter()

成功后 old_mode.exit()，记录日志/通知 
ArduPilot Firmware Download
+2
ArduPilot Firmware Download
+2

同时初始化阶段会设置初始模式：
Plane::init_ardupilot() 里有 set_mode(mode_initializing, ...)，并进一步 set_mode_by_number(g.initial_mode...)。 
ArduPilot Firmware Download
+1

5) “起飞 → 定点”在固定翼语境下怎么对应？（建议你用 Loiter 盘旋定点）

固定翼没有多旋翼那种“原地悬停”，通常“定点”对应的是：

Loiter：围绕一个点盘旋保持

或 Guided：引导到点并保持（实现上仍会是盘旋/航迹保持）

所以你要读的默认链路，我建议锁定为：

起飞 →（进入 AUTO / TAKEOFF）→ 达到条件 →（进入 LOITER 或继续 AUTO 航线）

结合上面的调度与模式框架，把“起飞到定点”翻译成调用链就是：

模式切换发生（RC 模式开关 / GCS 指令 / 任务触发）
Plane::set_mode(...) → new_mode.enter() 
ArduPilot Firmware Download
+1

每个控制周期（FAST_TASK）
update_control_mode() → control_mode->update() 
ArduPilot Firmware Download

导航与目标更新（10Hz 典型）
调度表里有 navigate、update_speed_height、update_alt 等任务持续更新目标与控制量 
ArduPilot Firmware Download

姿态/舵面控制与输出
stabilize（算姿态控制输出）→ set_servos（写入舵机/油门输出） 
ArduPilot Firmware Download

你如果要“只用一条线串起来”：
模式更新（决定目标）→ 导航（算航向/航迹）→ 速度高度控制（算油门/俯仰需求）→ 稳定（算舵面）→ 输出（写舵机）
这些都由 scheduler 按频率驱动，核心入口就是 scheduler_tasks[]。

6) 给你一条“阅读路线”（按你 SpringBoot 的习惯从骨架到业务）

第 1 段：把“容器/入口/生命周期”吃透

ArduPlane/ArduPlane.cpp：找到 AP_HAL_MAIN_CALLBACKS(&plane) 
ArduPilot Firmware Download

AP_HAL_Main.h：看宏如何生成 main & 交给 hal.run 
GitHub

AP_Vehicle.h：确认 setup/loop 的契约 
ArduPilot Firmware Download

第 2 段：把“调度器=调用链地图”吃透
4. ArduPlane/Plane.cpp：只盯 scheduler_tasks[]，把 FAST_TASK + 关键 10Hz/50Hz 任务记下来 
ArduPilot Firmware Download

第 3 段：把“模式框架=策略/状态机”吃透
5. ArduPlane/system.cpp：只读 set_mode() 的切换模板 + 初始化如何设置初始模式 
ArduPilot Firmware Download
+2
ArduPilot Firmware Download
+2

6. Plane::update_control_mode()：确认 control_mode->update() 每周期必跑 
ArduPilot Firmware Download

第 4 段：落到“起飞→Loiter”的具体模式实现
7. 找到 mode_takeoff、mode_auto、mode_loiter 对应的 enter()/update()（你已经知道入口点就是 control_mode->update() 了）
