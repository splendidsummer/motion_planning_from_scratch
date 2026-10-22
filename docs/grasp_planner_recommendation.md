# 这次桌面抓取用哪个规划器

拼接顺序和每一段查到哪里，见 [grasp_trajectory_workflow.md](grasp_trajectory_workflow.md)。逆解先于规划，见 [kdl_ompl.md](../tutorials/kdl_ompl.md)。按第一条能走的路排序，见 [performance.md](../tutorials/performance.md)。

> **建议：** `--planner RRTConnect`，每一段 `--time 1`。保持 `plan_between()` 先试关节直线。直线无碰就用直线，被挡住才把关节角交给 OMPL。

```bash
python3 docs/scene_gento_skye.py --planner RRTConnect --time 1
```

脚本默认已经是 `RRTConnect`，`--time` 默认 8 秒。8 秒对这个规划器偏长。

## 这三次请求在做什么

`choose_arm()` 留下一条手臂，7 个关节。预抓取、抓取、撤退各来一次：

1. `/compute_ik` 把夹爪位姿解成关节角。肘在身后、或者构型已经撞上的解丢掉。逆解失败，这一段规划不会开始。
2. `interpolated_motion()` 走关节空间直线，每步约 0.08 rad。整段无碰，这条直线就是轨迹。
3. 直线被挡住，`plan_group()` 才发 `/plan_kinematic_path`。每个关节一条 `JointConstraint`，容差 ±0.05 rad。

预抓取到抓取只有几厘米。这段直线无碰时，采样树容易绕开去扫到球，不该换一个更慢的规划器来做它。规划器主要负责从 home 绕到预抓取，有时还有撤退。

三条路都只要一条不撞桌面和球的关节折线。短不短先不管。末端也不会因为换了规划器就沿直线靠近方块。

## 为什么是 RRTConnect

按第一条可行路径出现要多久，这几个名字的顺序是：

1. `RRTConnect`
2. `BiTRRT`
3. `RRT`
4. `TRRT`
5. `BITstar`
6. `RRTstar`
7. `PRM`
8. `PRMstar`

`RRTConnect` 从起点和目标容差各长一棵树，每个新步查 1 条边，接到就停，剩下的时间不用。这台手臂上没有实测毫秒，顺序来自停止规则和每个新点要查几条边。手反复顶到桌沿或球时，失败的连接步都做了碰撞检查，它可以晚到和单向 `RRT` 差不多。1 秒里还没有路，多半是通道被挡住。

## 这次先不用的名字

| 名字 | 这次抓取 |
|---|---|
| `RRT`、`TRRT` | 一棵树自己走到目标，通常晚于 `RRTConnect`。这张桌子和球没有代价场，`TRRT` 多出来的检验几乎不过滤。 |
| `BiTRRT` | 停止规则和 `RRTConnect` 相同，每一步多一次几乎不过滤的代价检验。 |
| `RRTstar` | 三条路都会把 `--time` 用完来 rewire。单臂约 1000 个点时每个新点大约看 24 个近邻，第一条路也更晚。磨短的是关节折线。 |
| `PRM`、`PRMstar` | 三次 `plan()` 各建一张网。MoveIt 一次调用结束就把图丢掉，多查询的好处用不上。图连通之后才有第一条路。`PRM` 每个新点查固定的 9 条；`PRMstar` 在单臂 1000 个点时查 21 条，同样个数的里程碑更久。 |
| `BITstar` | 缩短路径通常比 `RRTstar` 快，但找到第一条路之后仍搜索到时间用完。不在 `scene_gento_skye.py` 的 `--planner` 列表里。每批固定 100 个样本。 |
| `FITstar` | 和 `BITstar` 同一类。还没有解时每批 199 个点；解变短之后下一批才变少，帮不上「尽快有一条能走的路」。不在 `--planner` 列表里。 |
| `AnytimePathShortening` | 会把 `--time` 用完。时间很短时，往往只拿到内层那条 `RRTConnect`。给数秒，短的是关节折线。这次要的就是那条能走的路，直接用内层。 |

投影、推迟检查那一组（`KPIECE`、`BKPIECE`、`LBKPIECE`、`SBL`、`EST`）改的是树往哪长。这次点到点不靠它们。

以后如果愿意等满时间，而且希望关节折线更短：每批点数跟着椭球变，用 `FITstar`；每批固定 100、启发式只用到终点的直线距离，用 `BITstar`；脚本里现在选得到的，用 `RRTstar` 或 `AnytimePathShortening`。同一张网要回答很多个起终点、并且网留得下来时，再用 `PRM`。

## 换规划器改变不了的

夹爪张开、闭合是直接插入的点，不调用 OMPL。方块在 `carry_after` 之后才被移到两个 TCP 的中点，规划 `lift` 时它还在桌面上。五段接成一条轨迹之后，没有再做整段碰撞检查。回放停在 RViz，不发给真机。这些边界写在 [grasp_trajectory_workflow.md](grasp_trajectory_workflow.md)。
