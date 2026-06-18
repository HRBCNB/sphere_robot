# 当前实现说明

本文档记录当前代码状态和常用命令，避免保留过多历史调试过程。

## 1. 核心实现

### ROLL / JUMP 混合 A*

A* 节点带有运动模式：

```text
ROLL：普通滚动
ROLL_OVER：低障碍滚越
JUMP：跳跃越障
DETOUR：高障碍绕行
```

基本判定：

```text
obstacle_height <= roll_over_height
  -> ROLL_OVER

roll_over_height < obstacle_height <= max_jump_h
  -> 尝试 JUMP

obstacle_height > max_jump_h 或跳跃不可行
  -> ROLL 绕行
```

### Direct A* 到 B-spline

`astar_only:=false` 时，direct A* 路径会参数化为 B-spline。

当前重点参数：

```text
direct_astar_sample_dist
direct_astar_smooth_iter
direct_astar_smooth_weight
direct_astar_roll_sample_dist
direct_astar_jump_sample_dist
direct_astar_jump_anchor_repeat
direct_astar_roll_anchor_repeat
direct_astar_allow_jump_impulse
direct_astar_time_realloc_max_iter
```

ROLL 转弯点重复锚定已改成可调：

```text
direct_astar_roll_anchor_repeat
```

主展示默认保留锚定；ROLL-only 跟踪展示设为 `0`，让轨迹更平滑。

### JUMP 冲量处理

JUMP 段允许较大瞬时加速度。参数：

```text
direct_astar_allow_jump_impulse=true
```

含义：当 direct A* 轨迹包含 JUMP 时，允许跳跃段冲量加速度，不再用无人机连续加速度约束把整条轨迹过度拉长。

注意：该模式适合规划展示，不适合用原无人机 SO3 控制器做跟踪展示。

## 2. 主要 launch

### JUMP 规划展示

```bash
roslaunch ego_planner midterm_global_demo.launch \
  astar_only:=false \
  use_control:=false \
  use_rviz:=true \
  show_whole_traj:=true \
  enable_local_replan:=false \
  enable_periodic_replan:=false \
  optimize_direct_astar:=false \
  direct_astar_allow_jump_impulse:=true
```

### ROLL-only 跟踪展示

```bash
roslaunch ego_planner midterm_roll_tracking_demo.launch
```

该 launch 用于生成更适合画跟踪图的平面绕障轨迹。

## 3. 跟踪画图

ROLL-only 展示建议使用新的 XY 脚本：

```bash
python3 src/planner/plan_manage/scripts/plot_roll_tracking_from_bag.py \
  midterm_roll_tracking.bag \
  --out-dir tracking_plots_roll_xy
```

输出：

```text
roll_tracking_position_xy.png
roll_tracking_velocity_xy.png
roll_tracking_speed_xy.png
roll_tracking_path_xy.png
roll_tracking_aligned_xy.csv
```

原始三轴脚本仍保留：

```bash
python3 src/planner/plan_manage/scripts/plot_tracking_from_bag.py BAG --out-dir tracking_plots
```

## 4. 重要日志

### 时间分配

```text
[EGOPlannerManager] direct A* ts debug: base_ts=..., vel_dt=..., acc_dt=..., selected=..., final_ts=...
```

用于判断 `ts` 是由原始 EGO、速度约束还是加速度约束决定。

### 最大间距

```text
[EGOPlannerManager] direct A* max gap: id=..., mode=..., gap=...
```

用于定位是哪段 ROLL/JUMP 影响时间估计。

### 可行性检查

```text
[B-spline feasibility] max_vel=...
[B-spline feasibility] max_acc=...
```

用于判断是否因为速度或加速度超限触发 `lengthenTime()`。

### 轨迹速度统计

```text
[EGOPlannerManager] traj speed stats: duration=..., mean_vel=..., max_vel=...
[traj_server] receive traj ..., physical_time_scale=..., expected_execute_duration=...
```

用于判断最终轨迹执行时间和参考速度是否适合展示。

## 5. 中期展示结论

建议明确区分两类验证：

```text
规划验证：展示 ROLL / JUMP / DETOUR 能力。
跟踪验证：只展示 ROLL-only 连续轨迹。
```

原因：当前控制器仍沿用无人机 SO3 控制器，不适合直接跟踪跳跃球的冲量 JUMP 段。后续工作应接入球形机器人专用控制器，将 JUMP 段转换为起跳速度、起跳角和飞行时间控制。
