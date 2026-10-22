# 这 13 个规划器

[桌面场景脚本](../docs/scene_gento_skye.py) 和 [关节目标规划脚本](../docs/plan_gento_skye.py) 的 `--planner` 只能取下面这些名字。它们都是 OMPL 的几何规划器：在关节空间里搜索通过碰撞检查的路径。[OMPL 配置](../gento_skye_moveit_config/config/ompl_planning.yaml) 中的 `AddTimeOptimalParameterization` 再为规划路径分配时间。

这里以带夹爪 Gento Skye 的 MoveIt / RViz 规划为背景。[RRT 教程](RRT.md) 已围绕当前桌子、方块、球障碍和双臂抓取演示重写，详细区分手臂路径检查与整段动画的验证边界。

先开演示：

```bash
ros2 launch gento_skye_moveit_config demo.launch.py
```

再换算法：

```bash
python3 docs/plan_gento_skye.py --group right_arm --planner RRTConnect --time 5
python3 docs/scene_gento_skye.py --planner RRT --time 8
```

RViz 里是同一组名字：MotionPlanning 面板的 Context 页，Planning Library 保持 `ompl`，下拉框选名字，再点 Plan。

## 符号

一条手臂的构型 q 是 7 个关节角，`both_arms` 是 14 维。夹爪不在手臂搜索变量中，但其碰撞网格仍随手臂运动，并按规划请求中固定的开度参与检查。自由空间 C_free 由关节限位、完整机器人状态、场景障碍及允许碰撞规则共同决定。

这里用关节空间欧氏距离 ‖q_a-q_b‖_2 解释构型之间的距离。OMPL 沿关节空间插值按分辨率检查碰撞，不是只检查 TCP 或路径端点；离散检查通过不等于连续扫掠体无碰撞的证明。

`range`（笔记里的步长 η）是树每次向外长的最大长度。没人设置时，OMPL 按状态空间对角线的一个比例自动填。

`--time` 写进 `allowed_planning_time`。找到一条就停的规划器会提前返回。会优化的规划器把这段时间用来换更短的路径。

## RRT 默认步长是否太大？

按本项目的关节限位盒子对角线，并假设 RRT 使用常见的 0.2 倍空间尺度默认机制，`range` 估算约为单臂 2.53、双臂 3.58。这不是从正在运行的 MoveIt 读取的参数。**双臂的 3.58 是 14 个关节角变化量的合成长度，不是每个关节都转 3.58 弧度，但它仍是较大的扩展尺度。**

假设 14 个关节的变化量绝对值都为 a，合成长度就是 √(14a²) = √14 × a。因此，用满双臂默认步长时，每个关节约变化 3.58 / √14 ≈ 0.96 rad ≈ 55°。除以 √14 只是在解释欧氏长度，并不说明这个步长很小；实际 RRT 也不会刻意平均分配步长。

默认取空间尺度的 20% 是通用启发式，不会根据当前障碍密度、连杆长度或双臂间隙来调整。大步长在开阔区域可能更快，但在拥挤区域可能频繁碰撞、扩展失败；小步长则可能增加节点数量和检查开销。`range` 不是控制周期内的运动量，也不是沿途碰撞检测的采样间隔，调小它不能替代碰撞检查分辨率的设置。

可对比 `range = 0.3、0.5、1.0` 与默认值。例如 0.5 在同样的等幅变化假设下，对应每关节约 7.7°。这些只是实验候选值，尚未验证哪个更适合这台机器人；应固定场景、起终点、碰撞检查分辨率和规划时间上限，多次测试成功率与规划耗时后再选。完整推导和取舍见 [RRT 教程](RRT.md)。

## 桌面场景的沿途检查与当前限制

桌面脚本通过 `/apply_planning_scene` 把桌板、底板、桌腿、方块和五个球作为碰撞对象提交给 MoveIt。白色桌板厚 5 cm、上表面 z = 0.36 m，方块边长 6 cm；两端不碰撞的关节运动，中途仍可能让指尖穿过桌板或前臂撞球。

[当前 OMPL 配置](../gento_skye_moveit_config/config/ompl_planning.yaml) 没有显式指定 `longest_valid_segment_fraction`、`maximum_waypoint_distance` 或 `range`。应核对运行时实际值，不能从桌板厚度直接换算一个通用关节空间检查间隔，也不能靠调低 [速度与加速度限制](../gento_skye_moveit_config/config/joint_limits.yaml) 来提高几何检查精度。

**当前演示不是已完整验证的抓取执行轨迹：** 三个双臂阶段分别规划，但夹爪开闭点是随后插入的，段间拼接未整体复核；撤退失败时可能临时移除方块重试，回放时移动方块也不等于附着物体规划。具体流程、参数含义和未验证部分见 [RRT 教程](RRT.md)。本项目当前使用 RViz 显示及模型回放，不连接真机控制器。

## 怎么读

哪五个值得先掌握，见 [prominent.md](prominent.md)。

| 名字 | 图 | 找到一条就停 | 文件 |
|---|---|---|---|
| `RRT` | 从起点长的树 | 是 | [RRT.md](RRT.md) |
| `RRTConnect` | 起点、终点各一棵树 | 是 | [RRTConnect.md](RRTConnect.md) |
| `RRTstar` | 带 rewire 的树 | 否，用完时间 | [RRTstar.md](RRTstar.md) |
| `PRM` | 与查询无关的路网 | 建图时间用完再查询 | [PRM.md](PRM.md) |
| `PRMstar` | 近邻个数随点数增加的路网 | 同上，路径会更接近最短 | [PRMstar.md](PRMstar.md) |
| `EST` | 按稀疏程度选点再局部采样 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#est) |
| `SBL` | 两棵树，边先假定能走 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#sbl) |
| `KPIECE` | 投影格子上偏向边界的树 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#kpiece) |
| `BKPIECE` | 两棵 KPIECE 树 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#bkpiece) |
| `LBKPIECE` | 两棵 KPIECE 树，碰撞检查推迟 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#lbkpiece) |
| `TRRT` | 带代价转移检验的树 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#trrt) |
| `BiTRRT` | 两棵这样的树 | 是 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#bitrrt) |
| `AnytimePathShortening` | 外包一层缩短 | 否，用完时间 | [KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#anytimepathshortening) |

点到点先用 `RRTConnect`。要更短的关节路径，把时间加到数秒，用 `RRTstar` 或 `AnytimePathShortening`。同一张路网要回答很多个起终点时用 `PRM`。`PRMstar` 的 k 随 1+1/d 略减，14 维上变贵的是把空间填满所需要的样本数。上面从 `EST` 起的名字是同一类树的改法，写在 [KPIECE、BKPIECE、LBKPIECE、SBL、EST、TRRT、BiTRRT、AnytimePathShortening、FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md)。

## 知情树

`BITstar` 和 `FITstar` 不在上面的 `--planner` 列表里，脚本和 RViz 的下拉框都选不到。Humble 自带的 OMPL 1.7 有 `BITstar`。`FITstar` 在本机这份 MoveIt 的 OMPL 接口里，写法见 [FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#fitstar)。

| 名字 | 图 | 找到一条就停 | 文件 |
|---|---|---|---|
| `BITstar` | 按批搜索的隐式路网，样本缩进椭球 | 否，用完时间 | [BITstar.md](BITstar.md) |

逆解和规划怎么接上，见 [kdl_ompl.md](kdl_ompl.md)。

对照：

| 文件 | 内容 |
|---|---|
| [prominent.md](prominent.md) | 哪五个值得掌握，其余名字改的是这个循环的哪一步 |
| [KPIECE、BKPIECE、LBKPIECE、SBL、EST、TRRT、BiTRRT、AnytimePathShortening、FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md) | 投影、稀疏扩展、代价转移、路径缩短、FIT* |
| [pros_cons.md](pros_cons.md) | 每个名字换来什么、付掉什么、什么时候不要用 |
| [performance.md](performance.md) | 第一条路多久出现、关节路径有多长、`--time` 用完还会不会变短 |
| [grasp_planner_recommendation.md](../docs/grasp_planner_recommendation.md) | 这次桌面抓取用 `RRTConnect`，其余名字先不用 |
| [efficiency.md](efficiency.md) | 碰撞检测次数、近邻、14 维上样本要铺多密 |
| [Dijkstra.md](Dijkstra.md) | PRM 查询时的 Dijkstra 和 A*，代价是关节长度 |
