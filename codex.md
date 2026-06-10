# A* Roll / Jump 混合规划修改说明

## 建议提交信息

```text
feat: 完善球形机器人 A* 滚动/跳跃混合规划

- 重构 A* 邻居扩展逻辑，区分 ROLL、ROLL_OVER、JUMP 和 ROLL detour
- 支持低障碍滚越、高障碍绕行、可跳低障碍提前起跳
- 新增跳跃起跳/落点安全余量，避免贴障碍边缘起跳或落地
- 修正 inflated obstacle shell 的高度判断，避免低障碍膨胀层被误判为必须绕行
- 将 A* 路径按球形机器人接地点表达，ROLL 段保持贴地
- 增加 A* 决策日志，输出 roll/jump/detour 代价和选择原因
- 增加 bend 混合测试场景，展示 roll、jump 和 detour 行为
- 修正 astar_only 调试模式的成功状态打印
- 关闭 RViz 默认无人机 mesh 标识，避免与球形机器人显示冲突
```

如果想写得更短，可以用：

```text
feat: 实现球形机器人 roll/jump 混合 A* 规划
```

## 当前规划策略

当前 A* 的目标是让球形机器人优先滚动，必要时跳跃：

1. 普通 free cell -> `ROLL`
2. 低障碍可滚越 -> `ROLL`，增加 `roll_over_penalty`
3. 低障碍不可滚但可跳 -> `JUMP`
4. 跳不过且可绕 -> 通过 free cell 形成 `ROLL detour`
5. 跳不过且绕不过 -> A* fail

这个顺序的含义是：能滚就滚，能绕就绕，跳跃只作为更严格、更贵的动作。

## ROLL / ROLL_OVER

ROLL 轨迹表示球形机器人的接地点，默认贴地：

- `astar_height = 0.0`
- ROLL 路径点 `z = 0.0`
- ROLL 控制点在 direct A* 初始化后也压回地面高度

低障碍滚越由 `roll_over_height` 控制。当前测试里低障碍高度不超过阈值时，仍然加入 `ROLL` 节点，只增加代价。

## JUMP 逻辑

JUMP 使用局部邻居方向，不再强行沿全局 start-goal 方向跳。

允许跳跃需要满足：

- 当前分支不在 detour 状态
- 落地后已经滚动超过 `min_roll_after_jump`
- 障碍高度高于 roll-over 阈值，但不超过 `max_jump_h`
- 跳跃方向和当前到目标方向足够一致：`dot(jump_dir, goal_dir) >= frontal_jump_cos`
- `isJumpFeasible(start, landing)` 通过
- landing 点在地图内、非占据、未进入 closed set

## 提前起跳和落点余量

之前的问题是：只有下一格已经是障碍时才尝试跳，所以起跳点和落点会贴着障碍边缘。

现在新增：

- `jump_takeoff_clearance`
- `jump_landing_clearance`

free cell 扩展时会向前看一小段距离，如果发现前方是“不能滚但可跳”的低障碍，就提前从当前点生成 `JUMP_EARLY` successor。

落点搜索也会从障碍后安全距离之外开始，避免刚越过障碍就落地。

当前默认值：

```text
jump_takeoff_clearance = 0.30
jump_landing_clearance = 0.30
```

如果起跳/落地仍然贴边，可以调到 `0.40` 或 `0.50`；如果跳得太早、弧线太长，可以调回 `0.20`。

## Detour 逻辑

detour 表示当前分支正在绕行障碍。

进入 detour 的情况：

- 前方障碍不能 roll-over
- 障碍不满足 jumpable 条件
- 或 jump feasible 检查失败

进入 detour 后：

- 禁止在同一个绕行过程中突然再次 JUMP
- 继续通过 free cell 或可滚越低障碍 ROLL
- 当路径回到 start-goal 主线附近，`lineDeviation <= detour_exit_deviation`，退出 detour

这样允许一条局部路径多段跳，但不会出现“已经决定绕第二个障碍，又在绕行中乱跳”的情况。

## inflated / raw obstacle 高度修正

之前日志出现过：

```text
raw_top=0.00 infl_top=0.10 roll=N jumpable=N
```

这说明 A* 看到的是膨胀层，但 raw obstacle 高度为 0，导致低障碍膨胀层被误判成既不能 roll 也不能 jump。

现在模式判断使用：

- raw 高度存在时，用 raw 高度
- raw 高度为 0 但 inflated 高度存在时，用 inflated 高度

这样低障碍的膨胀边缘也能正确判断为 `ROLL_OVER` 或 `JUMP`。

## A* only 调试模式

`astar_only:=true` 用来只看 A* marker，不发布 B-spline 控制轨迹。

现在成功时会打印：

```text
astar_only_plan_success=1
```

不会再出现 A* 已经找到路径但 `final_plan_success=0` 造成误解的情况。

推荐测试命令：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=true use_control:=false use_rviz:=true
```

完整规划和控制测试：

```bash
cd ~/workspace/sphere_robot
source devel/setup.bash
roslaunch ego_planner bend_jump_test.launch astar_only:=false use_control:=true use_rviz:=true
```

## 可视化调整

默认 RViz 配置已经关闭 `/odom_visualization/robot` 的无人机 mesh marker。

原因：当前对象已经改为球形机器人，原来的 hummingbird mesh 会误导观察。现在默认只看地图、A* 路径、轨迹和目标点。

## 当前效果判断

当前 A* only 效果已经基本达标：

- 低障碍能 roll-over 时优先 ROLL
- 可跳低障碍会提前起跳并留出落点余量
- 高障碍不硬跳，选择 ROLL detour
- 一条局部路径支持多段跳
- ROLL 段贴地，JUMP 段抬高

后续重点不应该继续大改 A*，而是验证 `astar_only:=false` 后：

- B-spline 是否把 A* 路径拉进障碍
- 跳跃落地后是否重新贴地
- 时间分配是否让跳跃和落地速度可控
- 控制执行是否能跟踪这条接地点轨迹
