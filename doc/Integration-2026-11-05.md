# 2026-11-05 整合分支说明

将团队 `DongTestCode` 的共用 Command、自动测试和 Foresight 参数，与鹏锦主干的自动瞄准、静摩擦补偿和新版射手标定合到同一分支。该分支供代码检查和后续实车验证，未装车，未认定为比赛版本。

## 合并来源

- 团队 `DongTestCode`：`0d61fdc6e8daa4b9dddd1a1255f0314e24e14c6c`，2026-11-05 12:57（北京时间）。
- `pengjinDaniel/FTC-16095-FatDragon-2027-Public` 的 `master`：`25df93bb5479549f26a52484c1beb1126e2c927b`，2026-11-05 12:12（北京时间）。包含 `c47d8af` 自动射球命令及之后的转向静摩擦补偿。
- 整合分支：`codex/integrate-dong-pengjin-20261105`。合并提交保留上述两条历史。

## 保留的实现与冲突处理

- 保留 `ShootCommand`、可无手柄调用的 `IntakeCommand`、`ShootAssistCommand` 的组合结构，以及 Intake 取消后持续停止的处理。
- `ShootAssistCommand` 调用共用 `ShootCommand`，距离改为鹏锦新版标定采用的 CELL 开口中心；保留新的自动瞄准控制器和静摩擦补偿。
- 保留鹏锦 `AutoShootCommand` 和原 `test` 七段路线，不强行改写为另一种底盘控制策略。该命令通过 `follower.manual()` 接管射击阶段底盘；不能与路径跟随并行。
- 保留 Dong 的 v8、第二测试 v3 和旧测试入口。v8 / v3 通过 `follower.hold()` 控制射击姿态，`ShootCommand` 仅占用射手、吸球、输送。
- 保留已填入的 Foresight 模型，以及鹏锦最新的底盘/定位配置。它们来自不同提交，需核对实车后再确认整套标定有效。
- 三个 Dong 射球测试（旧 v7、共用命令 v8、半二翻测试 v3）改为直接使用最新 `ShooterConstants` 电机方向，移除旧代码的额外 `.inverted()`，避免合并后重复反转。
- 上述三个测试的距离计算改为 CELL 开口中心，与新版射手距离表一致。路径坐标、摆车尺寸、射球时长未重新标定。
- 修正旧 `RedTwoTipAuto` 在 `CommandScheduler.reset()` 前持有旧实例的问题，改为 reset 后重新取得实例。
- 原有测试夹具适配新 Intake 调用；Foresight 缺失测试显式临时置空再恢复，新增已填配置可创建算法的验证。不删除缺失配置的保护。

## 程序入口

| Driver Station 名称 | 当前行为 | 备注 |
|---|---|---|
| Red 4 Ball Command Auto v8 | 到射击点、Pedro 保持姿态、共用命令送球、走第二段 | 优先用于验证共用射球链路；没有新增沿途取球 |
| Red Half Two Tip Test 2 v3 | 直行入场、射球、恢复朝向、下一点停三秒、跨场曲线 | 路径速度比例 0.35；只有一次射球，不代表两次 TIP 完成 |
| test | 原七段路线、三次射球与部分路径吸球 | 射球由 AutoShootCommand 控制底盘；射球超时后仍继续下一段 |

原有 RedFourBallCornerTest / RedTwoTipAuto 保留。不要仅凭相似菜单名判断正在运行哪条路线。

## 本地验证

- `:TeamCode:assembleDebug`：通过，生成调试 APK。
- 以下离线检查通过：`AutoAimPidfCheck`、`PracticeBlueCheck`、`RtCalibrationCheck`、`HiveGeometryCheck`、`TestAutoCheck`、`PedroTuningCheck`、`SharedShootMergeCheck`。
- 检查覆盖手动输入、静摩擦补偿、瞄准与视觉、真实调度器/输送计时、共用命令完成/超时/取消、吸球停止后的 periodic、三条 Dong 自动的距离基准。硬件输入使用替身，不验证真实运动、制动、命中或整场耗时。
- **`ShooterDistanceCheck` 未通过**：新版 `kF = 0.00058` 与 `1745 ticks/s` 目标产生约 `1.0121` 的前馈输出，达到 `maxPower = 1`，未满足原检查要求的调节余量。用合并前鹏锦 `c47d8af` 的编译类复现了同样失败；`25df93b` 没有再改射手。保留最新参数和失败断言，等待现场确定标定，不为通过检查猜改参数。

复现（先设置合适的 `JAVA_HOME`、Android SDK 路径；首次需下载依赖）：

```sh
python3 local-tests/run_integration_checks.py --offline
```

脚本构建 APK，使用实际依赖运行检查，日志在 `build/host-checks/`。默认包括上述失败检查，因此当前预期退出码为 1；不能报告全套检查通过。去掉 `--offline` 可让 Gradle 下载缺少的依赖。

## 待队友与实车确认

1. 按核心控制改动约定由两位非作者检查，其中至少一位了解实车；本次 AI 检查不代替批准。
2. 确认 Pinpoint 偏置（当前 X=98.5 mm、Y=-74 mm）、方向、车身尺寸和摆车基准，回归定位与 Foresight 制动。
3. 确认统一后的射手实际转向、距离测量到开口中心、转速/俯仰和前馈余量；计时送球完成不代表球已命中或完成 TIP。
4. 第二测试历史记录曾撞左墙、射球未进。保留的 v3 有路径降速、位置估计下的墙边余量检查，但不是独立测距防撞；定位/制动仍需实测。
5. 鹏锦 AutoShootCommand 的位置保持开关仍为 false；保留原实现，不将其当作已验证的主动位置锁定。共享瞄准参数修改后回归手动。
6. 对选定测试入口分别验证到位、瞄准、送球、取消/STOP 和下一段，再记录整条路线结果。本次未连接或启动车辆。
