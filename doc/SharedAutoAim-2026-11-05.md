# 易知路线接入共用射球与自动瞄准

本次将易知的两条自动测试路线接入鹏锦的瞄准控制，射手、吸球、输送统一由 `ShootCommand` 配合。手动继续通过 `ShootAssistCommand` 使用同一个 `ShootCommand`。代码已实现、构建及相关离线行为检查通过；尚未装车或取得实车验收。

分支：`codex/shared-auto-aim-20261105`。基于首次合并提交 `785e3fc9bbb082d241a7bc9aa5bd39a440944a97`；两端来源仍为 Dong `0d61fdc6e8daa4b9dddd1a1255f0314e24e14c6c`、鹏锦 `25df93bb5479549f26a52484c1beb1126e2c927b`。本次未修改硬件标定常量或路径坐标。

## 同学从哪里开始

Driver Station 的三个入口均在 `Test` 分组：

| 建议顺序 | 菜单名称 | 内容 |
|---|---|---|
| 1 | `Red Fixed Shot Shared Aim v1` | 固定点瞄准、射球，不走路径；START 后会自动转向 |
| 2 | `Red 4 Ball Shared Aim v9` | 原 v8 的两段路径，中间使用新的共用瞄准射球 |
| 3 | `Red Half Route Shared Aim v4` | 原半二翻 v3 的路径：第一点射球、恢复朝向、下一点停三秒、跨场 |

固定点入口默认摆在半场路线的第一射击点：Pedro 坐标 `(53.8928, 31.5834)` 英寸，吸球口朝向 `270°`；相对场地中心为 `(-18.1072, -40.4166)` 英寸。必须让实车摆放与这个初始位置一致。`setPose()` 只是设置位置估计，不能验证真实摆车是否正确。

v9 的起点与原 v8 相同，v4 的起点与原 v3 相同；对应界面显示摆车位置和 Build。v4 仍只射一次球，不能据菜单名称或流程结束认定完成两次有效 TIP。它保留路径速度比例 `0.35`、原路径方向检查与估计位置下的左墙余量检查。

三个入口默认 `feedDurationMs=2000`、射击总时限 13000 ms，`waitForHigh=false`，延续这两条路线原来第一轮射击不等待 HIGH 的行为。需要验证 HIGH 等待时，在对应入口配置中显式启用并记录；它依赖当前有效视觉观测。

## 谁负责什么

```text
手柄 → ShootAssistCommand ─┐
                         ├→ ShootCommand → Shooter / Intake / Transit
路径流程 → AutoShootCommand┘
              └→ AutoHiveTarget / AutoAimController → Pedro manual 输出
```

- 路径阶段由 Pedro 跟随路线，同时按当前距离预热射手；没有沿路新增吸球动作。
- `AutoShootCommand.initialize()` 停止前一段路径/保持输出，保留射手预热。射击阶段不再调用 Pedro `hold()` 或开始下一条路径。
- 每轮先更新目标与瞄准状态，再执行公共射球命令。射击期间继续瞄准；机构输出、连续送球计时、总超时和结果由 `ShootCommand` 统一负责。
- 使用 FTCLib 组合命令管理公共命令的启动和结束，并继承三个机构的 requirements。公共子命令不能另外独立调度。Follower 本身不是 FTCLib 子系统，底盘独占仍由路线阶段保证。
- v9 射后用当前实测位置、朝向构造第二段起点；v4 射后由 Pedro 恢复原导出的第二段起点及 270° 朝向，稳定后才继续。
- 只有 `SUCCEEDED` 才继续。超时、取消或漂移中止均停止路线并显示原因。原小写 `test` 入口也改为射击失败后跳过全部后续动作，并加入全程 30 秒截止。

## 启动、失效和停止的具体约定

1. 射击启动先等待平移速度不超过现有 `positionLockMaxSpeedInchesPerSecond`（默认 2 in/s），连续 200 ms 后记录射击位置，再使用原自动瞄准锁定判定。速度是定位估计，不能替代实际观察。
2. `FeedPermission.ALLOW`：由瞄准锁定、射手转速、俯仰共同决定是否开始输送。未正式送球时 `WAIT_FOR_START` 显式取消输送准备；因此 HIGH 在反转准备中消失，会关门停输送。此等待保留预热及原先射击准备时吸球机构运转的行为。
3. 保留鹏锦的 HIGH 规则：它约束开始射击，正式进入 `FEEDING` 后不会仅因 HIGH 观测消失而撤回已经开始的送球。
4. 定位/目标无效：停止底盘、射手、吸球和输送，关闭挡门，清零连续送球计时。公共命令本身允许在同一总时限内恢复；两条路线另有更严格的保护，Pinpoint 失去 READY 或位置出现非有限数时直接终止整条路线。射击阶段拒绝射手距离标定时，也不输出底盘瞄准功率。
5. 相对射击记录位置偏移超过 1.5 in，或者反转/送球期间偏航超过 3°、平移速度超过上述阈值，返回 `ABORTED` 并停止。普通小幅偏航或飞轮转速波动仍遵循原已提交送球规则。
6. 送球时长从 `FEEDING` 开始，包含瞄准/准备等待的总时限由公共命令单独计时；超时结果为 `TIMED_OUT`。STOP/调度取消结果为 `INTERRUPTED`，机构停止状态在后续 periodic 中继续保持。
7. 两条路线从 START 开始计算全程 30 秒，包含等待 Pinpoint 的时间。各阶段上限并不保证总流程能在 30 秒内跑完，长等待会截断后续路线，需实测每段用时。

`DriveConstants.positionHoldCalibrated` 仍为 `false`，所以当前默认实现主动修正朝向，平移漂移依靠中止保护。不要把它当作已验证的主动锁位，也不要未经标定直接打开开关。目标距离和偏航仍依赖 Pedro 位置估计；本次没有加入视觉修正定位。

## 本地验证及剩余问题

- `:TeamCode:assembleDebug`：通过。
- 通过的检查组：`AutoAimPidfCheck`、`PracticeBlueCheck`、`RtCalibrationCheck`、`HiveGeometryCheck`、`TestAutoCheck`、`PedroTuningCheck`、`SharedShootMergeCheck`、`SharedAutoAimCheck`。
- 新增 50 项行为断言，使用真实 FTCLib 调度器、公共命令、Intake 和 Transit；覆盖预热、等待停稳、瞄准符号、转速/俯仰门槛、HIGH 失效、定位/距离失效及恢复、送球计时、超时、STOP、重复调度、父组合结束、漂移中止和两条路线的成功/失败交接。替身提供电机、传感器与位置数据；不验证真实运动或命中。
- `local-tests/Log.java` 仅给电脑测试提供 Android 日志桥接，不参与机器人 APK 构建。
- **`ShooterDistanceCheck` 仍失败**：继承基线的射手参数余量问题，`0.00058 × 1745 = 1.0121` 已超过 `maxPower=1`。没有猜改标定值或删掉断言，不能宣称全套检查通过。此前合并记录已在鹏锦合并前代码上复现同一失败。

复现（配置 Java 与 Android SDK 后）：

```sh
python3 local-tests/run_integration_checks.py --offline
```

完整运行预期仍因上述标定检查退出 1；每组实际结果在 `build/host-checks/results.json`，构建和检查日志也在该目录。

本地构建产物为 `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`，本次构建 SHA-256：`00b3604429de767581b5e0cdd22822bb8acd437c19638827919a815cd1fe6ade`。重新构建后的产物请重新记录校验值。

实车尚待确认：定位与制动、固定点瞄准时的位移、射手距离/转速/俯仰及功率余量、真实送球时长、STOP、路径衔接和全程耗时。按团队现行核心控制约定，正式合入主干前需两位非作者检查，其中至少一位了解实车；这里没有代填任何人的批准。原方案可在基线提交 `785e3fc` 中对照。
