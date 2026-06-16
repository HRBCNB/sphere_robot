# 球形机器人跳跃避障规划说明

本项目用于设计一个可滚动、可跳跃避障的球形机器人，作为硕士毕业设计实验平台。

本文档只记录当前实现的功能、实现方式、参数含义和验证方法，不记录提交日志。

使用docker环境之前先在宿主机执行:
```bash
xhost +local:root
```

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
- 跳跃弧线跨过 raw 障碍水平投影时，高度需要高出障碍顶部至少 `jump_vertical_clearance`
- 落点在地图内且不是 occupied

跳跃路径会插入抛物线采样点，JUMP 点在 RViz 中以橙红色点显示。

### 提前起跳和落点安全余量

为了避免贴着障碍起跳或刚越过障碍就落地，当前支持：

```text
jump_takeoff_clearance
jump_landing_clearance
```

当 ROLL 扩展时，如果前方短距离内出现可跳障碍，会提前生成 `JUMP` successor。当前起跳触发前视距离约为：

```text
jump_takeoff_clearance + 2 * map_resolution
```

也就是障碍必须进入起跳安全余量附近才触发 JUMP，避免为了用满 `max_jump_d` 而过早起跳。触发后，A* 会继续沿跳跃方向扫描障碍厚度，落点最小距离按“障碍后沿 + `jump_landing_clearance`”计算，避免刚越过障碍就提前落地。

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

当前 direct A* 转 B-spline 前做了按模式分段处理，目标是让 ROLL 段更平顺，同时让 JUMP 段尽量保持 A* 原始抛物线：

- B-spline 输入点基于 raw A* 原始节点生成，保留原始绕障拐角，不再先用大间距采样丢掉拐角细节。
- ROLL 段送入 B-spline 前的最大局部采样间隔限制为 `min(direct_astar_sample_dist, 0.25)`。
- JUMP 段送入 B-spline 前的最大局部采样间隔限制为 `min(direct_astar_sample_dist, 0.05)`。
- JUMP 段直接使用 A* 生成的抛物线采样点，不再额外抬高 JUMP 点，也不再用后端控制点强行改 JUMP 高度。
- JUMP 采样点会重复插入作为锚点，JUMP 控制点模式映射扩大到邻域 `+-6`，避免相邻 ROLL 控制点把抛物线边缘拉回地面；当前 JUMP 锚点比 ROLL 更强，用于减少蓝色 B-spline 对橙色 A* 抛物线的切角。
- ROLL 点如果靠近不可滚越高障碍的 inflated 区，会在参数化前轻微向远离障碍方向预成形，给 B-spline 留出连续曲线安全余量。
- ROLL 段做轻量平滑和急转弯锚定；JUMP 段不参与这部分后端前平滑，保持 A* 抛物线形状。
- direct A* 中包含 JUMP 时，会放大初始时间间隔，优先通过时间拉长满足动力学约束，不通过几何拉扯改变跳跃形状。

### 最终碰撞检查

最终 B-spline 发布前会做碰撞检查。

当前策略：

- 进入 raw 实体障碍：拒绝轨迹
- 贴地 ROLL 遇到低障碍且满足 roll-over：允许通过
- 贴地 ROLL 进入不可滚越 inflated obstacle：拒绝轨迹
- 空中 JUMP 越过 `max_jump_h` 范围内的低障碍 inflated shell：允许通过，但实际采样点不能进入 raw 实体障碍
- 高障碍或非跳跃语义的 inflated 碰撞：拒绝轨迹

这样做是为了匹配 `use_inflate_for_jump=false` 的 A* 跳跃语义：JUMP 抛物线允许越过低障碍的膨胀壳，但不能穿进真实障碍体；ROLL 和高障碍仍保持严格检查。

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

当前 `default.rviz` 里的 Grid `Cell Size` 为 `1`，所以地面小方格每格代表 `1 m`。

颜色含义：

- 蓝色连续线：最终通过检查并发布的 B-spline 轨迹
- 橙红色连续线：被最终碰撞检查 reject 的失败候选 B-spline，不是可执行轨迹
- 洋红色点：当前这一帧 A* ROLL 路径点
- 橙红色点：当前这一帧 A* JUMP 路径点
- 淡紫色点/线：局部控制点或局部轨迹辅助显示
- 青绿色球：全局目标点，不是障碍，也不是轨迹点；bend 固定目标默认 `goal_z:=0.0`，显示在地面高度

判断标准：

- 只有红线没有蓝线：当前候选 B-spline 被拒绝，没有可执行最终轨迹
- 出现蓝线：当前规划结果已经通过最终检查并发布
- A* 点只显示当前重规划结果，不再累积历史粉色点；如果 RViz 中仍残留旧 marker，可以重启 RViz 或重新打开显示项清空
- 蓝线应避开 raw 实体障碍；JUMP 段允许越过可跳低障碍的 inflated shell

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
planning_horizon:=7.5
enable_local_replan:=false enable_periodic_replan:=false
optimize_direct_astar:=false
direct_astar_sample_dist:=0.8
direct_astar_smooth_iter:=2
direct_astar_smooth_weight:=0.45
direct_astar_jump_clearance:=0.25
direct_astar_time_scale:=1.35
```

含义：

- `astar_only`：只显示 A*，不发布 B-spline 轨迹
- `show_local_traj`：显示局部控制点/局部轨迹
- `show_whole_traj`：显示完整 B-spline 采样轨迹
- `whole_traj_sample_step`：完整轨迹采样间隔
- `planning_horizon`：单次规划视距；`midterm_global_demo.launch` 默认 `18.5`，并配合更大的 `astar_pool_size_*` 搜索池使用
- `enable_local_replan`：局部重规划总开关；中期固定轨迹演示默认关闭，后续要展示在线局部重规划时再打开
- `enable_periodic_replan`：周期触发重规划开关；只有 `enable_local_replan=true` 时才会生效
- `optimize_direct_astar`：是否打开旧 rebound optimizer，默认不建议打开
- `direct_astar_sample_dist`：direct A* 路径转 B-spline 前的基础采样距离；实际 B-spline 输入会对 ROLL/JUMP 再做保护采样上限
- `direct_astar_smooth_iter`：direct A* ROLL 点轻量平滑迭代次数
- `direct_astar_smooth_weight`：direct A* ROLL 点轻量平滑权重
- `direct_astar_jump_clearance`：A* 生成 JUMP 抛物线时相对障碍顶部保留的基础高度余量；当前 B-spline 转换不再额外抬高 JUMP 段
- `direct_astar_time_scale`：direct A* 转 B-spline 时的时间放大系数；包含 JUMP 时默认 `1.35`，`midterm_global_demo.launch` 为了展示速度使用 `1.25`；值越大执行越慢但越保守，值越小轨迹更快但更容易速度/加速度超限
- `astar_pool_size_x/y/z`：A* 搜索池尺寸。默认小场景为 `100/100/100`；中期大地图使用 `240/140/50`，用于支持约 18 m 的一次性固定轨迹搜索

### 固定轨迹执行速度

中期推荐的 `midterm_global_demo.launch` 为固定轨迹跟踪演示使用的默认速度参数为：

```text
manager/max_vel = 3.0
manager/max_acc = 4.5
direct_astar_time_scale = 1.25
```

如果机器人执行仍然太慢，优先调大 `max_vel` 和 `max_acc`；如果包含 JUMP 的轨迹整体被拉得过慢，再小幅降低 `direct_astar_time_scale`。不建议一次降得太多，因为最终 B-spline 仍需要满足速度和加速度可行性检查。

### 规划参数

常用规划参数：

```text
planner/max_jump_h
planner/max_jump_d
planner/jump_penalty
planner/jump_takeoff_clearance
planner/jump_landing_clearance
planner/jump_vertical_clearance
planner/roll_over_height
planner/roll_over_penalty
planner/frontal_jump_cos
planner/use_inflate_for_jump
```

参数含义和调参影响：

- `planner/max_jump_h`：允许跳跃跨越的最大障碍高度，也是 JUMP 抛物线高度检查的重要上限。调大后能跳过更高障碍，弧线也更容易越过墙顶；过大时会让不该跳的障碍也被当成可跳目标。
- `planner/max_jump_d`：单次 JUMP 的最大水平距离。调大后能跨过更宽的障碍或更远的间隙；过大时搜索空间变大，也可能产生过长、过早的跳跃候选。
- `planner/jump_penalty`：A* 中使用 JUMP 的额外代价。调大后规划器更倾向 ROLL 或绕行；调小后更容易选择跳跃。
- `planner/jump_takeoff_clearance`：起跳前与障碍前沿保留的水平安全距离。调大后起跳点更靠前，留给起跳准备的距离更长；过大时会显得起跳太早。
- `planner/jump_landing_clearance`：落点相对障碍后沿保留的水平安全距离。调大后落点离墙更远，不容易刚越过障碍就落地；过大时可能因为 `max_jump_d` 不够而找不到可行跳跃。
- `planner/jump_vertical_clearance`：JUMP 抛物线跨过 raw 障碍水平投影时，轨迹高度需要高出障碍顶部的最小余量。调大后弧线更不容易贴墙顶；过大时需要同步增大 `max_jump_h` 或 `max_jump_d`，否则 A* 会判定跳不过。
- `planner/roll_over_height`：允许 ROLL 滚越的低障碍高度阈值。障碍顶部不超过 `astar_height + roll_over_height` 时按滚越处理，不触发 JUMP。调大后更多障碍会被当成可滚越；过大时会把实际应该跳/绕的障碍误判为可滚。
- `planner/roll_over_penalty`：ROLL_OVER 的额外代价。调大后即使能滚越，A* 也会更愿意绕开低障碍；调小后会更积极直接滚过低障碍。
- `planner/frontal_jump_cos`：JUMP 方向与当前朝目标方向的一致性阈值，取值越接近 `1.0` 越要求正向跳。调大后跳跃更朝向目标、不容易侧跳；调小后允许更多斜向跳跃，但可能出现绕行中不稳定的跳跃候选。
- `planner/use_inflate_for_jump`：JUMP 可行性是否使用 inflated obstacle。设为 `true` 时跳跃更保守，会把膨胀壳也当成障碍；设为 `false` 时主要参考 raw obstacle，允许 JUMP 越过低障碍的 inflated shell，但最终轨迹仍不能进入 raw 实体障碍。

当前 bend 测试场景中，`use_inflate_for_jump=false`，跳跃可行性主要参考 raw obstacle，避免 inflated shell 让跳跃过度保守。

Bend 静态演示中为了避免跳跃段贴墙，当前覆盖了更保守的跳跃水平余量：

```text
planner/max_jump_h = 0.70
planner/max_jump_d = 1.8
planner/jump_takeoff_clearance = 0.45
planner/jump_landing_clearance = 0.50
planner/jump_vertical_clearance = 0.16
```

`max_jump_h` 决定 A* 抛物线峰值高度；`jump_vertical_clearance` 是 A* 层的越障高度余量。当前 bend 场景把 `max_jump_h` 提高到 `0.70`，避免 JUMP 弧线视觉上贴着墙顶。A* 会检查 JUMP 抛物线跨过 raw 障碍水平投影时，轨迹高度是否高于障碍顶部一段距离，避免 A* 自己给出贴墙擦边的跳跃点。

如果看到 JUMP 落点仍贴近墙后侧，优先调大 `jump_landing_clearance`；当前落点余量已经从障碍后沿开始计算，不是从障碍前沿计算。如果看到 JUMP 弧线贴着墙顶或穿过墙体边缘，优先调大 `jump_vertical_clearance`；如果调大后跳不过，再同步增大 `max_jump_h` 或 `max_jump_d`，不要只放宽最终碰撞检查。

## 如何运行验证

### 图形界面授权

如果当前是以 root 身份进入容器或运行环境，并且需要打开 RViz，进入环境后先执行：

```bash
xhost +local:root
```

该命令用于允许 root 用户连接宿主机 X11 显示，否则 RViz 可能无法启动或无法显示窗口。

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

### 中期大地图固定轨迹展示场景

`midterm_global_demo.launch` 是当前中期推荐入口。它按“飞机/机器人开局已知完整静态地图、固定终点、一次规划、无周期重规划”的口径配置。默认起点由仿真器给出：

```text
start = (-18.0, 0.0, 0.0)
goal  = (0.0, 0.0, 0.0)
```

地图长度约 18 m，障碍从左到右分成几个清晰阶段：

1. 极低宽障碍：展示 `ROLL_OVER`，机器人保持滚动通过。
2. 低跳墙和两侧护栏：展示正向 `JUMP`，A* 点中会出现橙红色跳跃段。
3. 中央高障碍：高度超过 `max_jump_h`，展示绕行而不是强行跳跃。
4. 绕行后短低墙：展示回到主线后的第二次 `JUMP`。
5. 窄通道和侧向视觉障碍：增强场景层次，同时不堵死主通道。

为了支持这张大地图一次性规划，`midterm_global_demo.launch` 将 A* 搜索池从默认 `100 x 100 x 100` 提高到：

```text
astar_pool_size_x = 240
astar_pool_size_y = 140
astar_pool_size_z = 50
```

推荐静态轨迹演示命令：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner midterm_global_demo.launch use_control:=false use_rviz:=true show_local_traj:=true show_whole_traj:=true enable_local_replan:=false enable_periodic_replan:=false
```

推荐固定轨迹跟踪演示命令：

```bash
roslaunch ego_planner midterm_global_demo.launch astar_only:=false use_control:=true use_rviz:=true show_whole_traj:=true enable_local_replan:=false enable_periodic_replan:=false
```

如果只想看 A* 决策，不看 B-spline：

```bash
roslaunch ego_planner midterm_global_demo.launch astar_only:=true use_control:=false use_rviz:=true
```

观察重点：低障碍直接滚越、低墙跳跃、高墙绕行、绕行后再次跳跃。这个场景就是中期用于展示的“大一点的已知地图一次性轨迹”。

### 完整规划 RViz 验证

用于验证 A* 转 B-spline 后是否能发布最终轨迹：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=false use_control:=false use_rviz:=true show_local_traj:=true show_whole_traj:=true planning_horizon:=7.5 enable_local_replan:=false enable_periodic_replan:=false optimize_direct_astar:=false
```

观察重点：

- 是否出现蓝色最终 B-spline
- JUMP 段蓝线是否基本贴合 A* 抛物线点
- 蓝线是否保持在 raw 实体障碍外
- 是否只出现红色失败候选轨迹
- A* 点和 B-spline 之间偏差是否过大

### 无 RViz 日志验证

用于快速确认规划是否进入执行状态：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=false use_control:=false use_rviz:=false show_whole_traj:=true planning_horizon:=7.5 enable_local_replan:=false enable_periodic_replan:=false
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


### 目标点和规划视距

RViz 里的青绿色球是目标点。之前手动目标回调会把目标 z 强制写成 `1.0`，所以目标球会漂在空中；现在目标 z 使用消息里的值，`midterm_global_demo.launch` 默认是 `goal_z:=0.0`。

中期展示不使用在线重规划。`midterm_global_demo.launch` 将起点设为 `(-18.0, 0.0, 0.0)`，终点设为 `(0.0, 0.0, 0.0)`，`planning_horizon=18.5`，并增大 A* 搜索池，保证目标点位于一次规划范围内。

`show_whole_traj` 显示的是当前一次性发布的 B-spline 轨迹。不要把远处目标直接设到几十米外再关闭重规划，否则系统只能规划到局部目标，甚至可能出现 `Ran out of pool, index=...`。如果后续要做更长路线，应扩展搜索池或设计分段轨迹，而不是在中期展示里打开周期重规划。

### 周期重规划说明

EGO 原始 FSM 在 `EXEC_TRAJ` 中会根据当前执行时间自动切到 `REPLAN_TRAJ`，用于真实机器人或闭环仿真中的在线局部重规划。

当前固定轨迹演示默认：

```text
enable_local_replan = false
enable_periodic_replan = false
```

`enable_local_replan` 是总开关，关闭后新目标触发、周期触发和安全检查触发的局部重规划都会被挡住。`enable_periodic_replan` 只控制按时间周期触发的那一路。当前中期展示两个都关闭，规划成功后会保持当前发布轨迹，不会因为时间推进反复刷：

```text
[FSM]: from EXEC_TRAJ to REPLAN_TRAJ
[FSM]: from REPLAN_TRAJ to EXEC_TRAJ
```

中期展示保持 `enable_local_replan=false`、`enable_periodic_replan=false`。Safety 轨迹检查仍会检测风险；如果局部重规划关闭，遇到突发碰撞风险会进入急停而不是切到 `REPLAN_TRAJ`。

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

- direct A* 到单条三次 B-spline 仍是近似转换，极端窄通道、连续急转弯或很短的 ROLL/JUMP 连接段仍可能出现形状偏差。
- 当前实现是最小分段处理：JUMP 段锁住 A* 抛物线，ROLL 段做预成形/平滑保护；旧 rebound optimizer 默认仍关闭。
- JUMP 允许穿过低障碍 inflated shell，但必须避开 raw 实体障碍；因此 RViz 中需要结合实体障碍和膨胀显示一起判断。
- 更合理的长期方案是生成真正的分段轨迹：ROLL 段贴地 B-spline，JUMP 段独立抛物线，ROLL/JUMP 连接处单独做短过渡。

## 文档维护约定

后续只要修改工程代码、launch 参数、可视化样式或调试策略，都需要同步更新本文档。

每次更新至少说明：

- 实现了什么功能
- 关键实现方式
- 如何 launch 验证
- 当前已知问题
