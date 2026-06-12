# 球形机器人跳跃避障规划说明

本项目用于设计一个可滚动、可跳跃避障的球形机器人，作为硕士毕业设计实验平台。

本文档只记录当前实现的功能、实现方式、参数含义和验证方法，不记录提交日志。

## 已实现功能

### Roll / Jump 混合 A* 规划

当前 A* 规划器面向球形机器人接地点规划，优先滚动，必要时跳跃：

1. 普通空闲区域使用 `ROLL`。
2. 低障碍可滚越时仍使用 `ROLL`，但增加滚越代价。
3. 障碍不能滚越但可跳过时使用 `JUMP`。
4. 障碍太高或跳跃不可行时，使用 `ROLL` 绕行。
5. 跳不过且绕不过时，A* 搜索失败。

ROLL 段默认贴地，使用 `astar_height` 作为接地点高度。当前测试场景中该值为 `0.0`。

### 低障碍滚越

低障碍滚越由 `roll_over_height` 控制。

当障碍顶部高度不超过：

```text
astar_height + roll_over_height
```

该位置允许作为 `ROLL` 通过，只增加 `roll_over_penalty`。这样低矮障碍不会被误判为必须跳跃或绕行。

### 跳跃规划

JUMP 使用局部邻居方向生成，不强制沿全局 start-goal 方向跳。

允许跳跃需要满足：

- 当前分支不在 detour 状态
- 距离上一次落地已经滚动超过 `min_roll_after_jump`
- 障碍高度高于滚越阈值，但不超过 `max_jump_h`
- 跳跃方向与当前到目标方向足够一致，满足 `frontal_jump_cos`
- 起跳点、落点和跳跃弧线通过可行性检查
- 落点在地图内且不是 occupied

跳跃路径会插入抛物线采样点，JUMP 点在 RViz 中以橙红色点显示。

### 提前起跳和落点安全余量

为了避免贴着障碍起跳或刚越过障碍就落地，当前支持：

```text
jump_takeoff_clearance
jump_landing_clearance
```

当 ROLL 扩展时，如果前方短距离内出现可跳障碍，会提前生成 `JUMP` successor。落点搜索也会从障碍后方安全距离以外开始。

### Detour 绕行逻辑

detour 表示当前分支正在绕过不可滚越、不可跳或跳跃检查失败的障碍。

进入 detour 后：

- 禁止在同一个绕行过程中突然再次跳跃
- 继续通过 free cell 或可滚越低障碍滚动
- 当路径回到 start-goal 主线附近时退出 detour

这样可以支持一条路径中有多段跳跃，同时避免绕行中出现不稳定的跳跃决策。

### Direct A* 到 B-spline

在 `astar_only:=false` 时，系统会把 direct A* 路径参数化为 B-spline 轨迹。

当前默认：

```text
optimize_direct_astar = false
```

也就是 direct A* 初始化路径会跳过旧的 rebound optimizer，尽量保持 A* 的 ROLL / JUMP / detour 结构。

原因是旧 rebound optimizer 主要按普通飞行轨迹处理，容易破坏球形机器人的贴地 ROLL 和 JUMP 模式。

### 最终碰撞检查

最终 B-spline 发布前会做碰撞检查。

当前策略：

- 进入 raw 实体障碍：拒绝轨迹
- 低障碍满足 roll-over：允许通过
- 进入 inflated obstacle：默认拒绝轨迹
- 低障碍满足 roll-over：允许通过
- 非贴地或真实进入障碍体：拒绝轨迹

当前不再允许最终蓝色 B-spline 穿过 inflated shell。这样 RViz 中只要出现蓝线，就应该和可视化障碍保持清楚分离。

如果轨迹被拒绝，终端会出现类似：

```text
[EGOPlannerManager] final B-spline hits inflated obstacle, reject traj. hit=(x, y, z), dt=...
```

### 轨迹可视化

当前 RViz 中主要显示：

- A* ROLL 点
- A* JUMP 点
- 最终 B-spline 轨迹
- 被拒绝的候选 B-spline
- 局部控制点/局部轨迹辅助显示

颜色含义：

- 蓝色连续线：最终通过检查并发布的 B-spline 轨迹
- 橙红色连续线：被最终碰撞检查 reject 的失败候选 B-spline，不是可执行轨迹
- 洋红色点：A* ROLL 路径点
- 橙红色点：A* JUMP 路径点
- 淡紫色点/线：局部控制点或局部轨迹辅助显示

判断标准：

- 只有红线没有蓝线：当前候选 B-spline 被拒绝，没有可执行最终轨迹
- 出现蓝线：当前规划结果已经通过最终检查并发布
- 蓝线应避开可视化障碍和 inflated obstacle，不应压进障碍显示范围

## 关键参数

### Launch 参数

`bend_jump_test.launch` 中常用参数：

```bash
astar_only:=false
use_control:=false
use_rviz:=true
show_local_traj:=true
show_whole_traj:=true
whole_traj_sample_step:=0.05
optimize_direct_astar:=false
direct_astar_sample_dist:=0.8
direct_astar_smooth_iter:=2
direct_astar_smooth_weight:=0.45
direct_astar_jump_clearance:=0.25
```

含义：

- `astar_only`：只显示 A*，不发布 B-spline 轨迹
- `show_local_traj`：显示局部控制点/局部轨迹
- `show_whole_traj`：显示完整 B-spline 采样轨迹
- `whole_traj_sample_step`：完整轨迹采样间隔
- `optimize_direct_astar`：是否打开旧 rebound optimizer，默认不建议打开
- `direct_astar_sample_dist`：direct A* 路径转 B-spline 前的采样距离
- `direct_astar_smooth_iter`：direct A* ROLL 点轻量平滑迭代次数
- `direct_astar_smooth_weight`：direct A* ROLL 点轻量平滑权重
- `direct_astar_jump_clearance`：JUMP 段相对障碍顶部保留的高度余量

### 规划参数

常用规划参数：

```text
planner/max_jump_h
planner/max_jump_d
planner/jump_penalty
planner/jump_takeoff_clearance
planner/jump_landing_clearance
planner/roll_over_height
planner/roll_over_penalty
planner/frontal_jump_cos
planner/use_inflate_for_jump
```

当前 bend 测试场景中，`use_inflate_for_jump=false`，跳跃可行性主要参考 raw obstacle，避免 inflated shell 让跳跃过度保守。

## 如何运行验证

### 只看 A*

用于验证 ROLL / JUMP / detour 决策是否合理：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=true use_control:=false use_rviz:=true
```

观察重点：

- 低障碍是否优先 ROLL / ROLL_OVER
- 可跳障碍是否出现 JUMP 点
- 高障碍是否绕行
- JUMP 起跳和落点是否留有安全距离

### 完整规划 RViz 验证

用于验证 A* 转 B-spline 后是否能发布最终轨迹：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=false use_control:=false use_rviz:=true show_local_traj:=true show_whole_traj:=true optimize_direct_astar:=false
```

观察重点：

- 是否出现蓝色最终 B-spline
- 蓝线是否保持在实体障碍外
- 是否只出现红色失败候选轨迹
- A* 点和 B-spline 之间偏差是否过大

### 无 RViz 日志验证

用于快速确认规划是否进入执行状态：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=false use_control:=false use_rviz:=false show_whole_traj:=true
```

通过标准：

```text
[FSM]: from WAIT_TARGET to GEN_NEW_TRAJ
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ
```

如果持续出现：

```text
final B-spline hits inflated obstacle
```

说明最终 B-spline 仍然没有通过碰撞检查。

### 测试旧 rebound optimizer

仅用于诊断，不建议作为稳定演示：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=false use_control:=false use_rviz:=true show_local_traj:=true show_whole_traj:=true optimize_direct_astar:=true
```

当前旧 rebound optimizer 对 ROLL / JUMP 模式理解不足，可能出现内部 A* 修复失败或轨迹模式被破坏。

## 日志说明

为减少终端刷屏，当前已注释掉大量 A* 详细调试输出，包括：

- A* 参数完整打印
- 每个 decision 的详细代价
- A* path 每个节点逐行打印
- jump span 统计
- direct A* 初始化细节
- B-spline 优化部分耗时输出

当前保留：

- FSM 状态跳转
- A* enter / exit
- 关键错误
- 最终碰撞 reject 提示
- 急停提示

如果需要重新打开 A* 细节调试，可以把 launch 中：

```xml
<param name="/ego_planner_node/planner/debug_decisions" value="false" type="bool"/>
```

改为：

```xml
<param name="/ego_planner_node/planner/debug_decisions" value="true" type="bool"/>
```

并取消代码中相关 `ROS_INFO/ROS_WARN` 注释。

## 当前已知限制

- direct A* 折线转 B-spline 后，绕障或跳跃过渡处仍可能产生切角。
- 当前保持最终 B-spline 对 inflated obstacle 的严格检查，因此某些切角候选可能只有红线、没有蓝线。
- 不建议使用大幅控制点强拉或放宽 inflated shell 检查来修复切角，因为 RViz 中轨迹形状或避障语义会明显变差。
- 更合理的后续方案是按模式分段生成轨迹：ROLL 段贴地，JUMP 段保留抛物线，ROLL/JUMP 连接处单独做短过渡。

## 文档维护约定

后续只要修改工程代码、launch 参数、可视化样式或调试策略，都需要同步更新本文档。

每次更新至少说明：

- 实现了什么功能
- 关键实现方式
- 如何 launch 验证
- 当前已知问题
