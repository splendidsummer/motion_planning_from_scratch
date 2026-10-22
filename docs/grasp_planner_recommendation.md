# 这次桌面抓取用哪个规划器

拼接顺序和每一段查到哪里，见 [grasp_trajectory_workflow.md](grasp_trajectory_workflow.md)。逆解先于规划，见 [kdl_ompl.md](../tutorials/kdl_ompl.md)。这只手臂上各个逆解库做到哪一步，见 [ik_solver_comparison.md](ik_solver_comparison.md)。按第一条能走的路排序，见 [performance.md](../tutorials/performance.md)。

> **建议：** 先用 `kdl_arm_ik.py` 对左右臂各求一次 7 轴逆解，留下更适用的那只手，再 `--planner RRTConnect`，每一段 `--time 1`。保持 `plan_between()` 先试关节直线。直线无碰就用直线，被挡住才把这组关节角交给 OMPL。

```bash
/usr/bin/python3 kdl_arm_ik.py \
  --xyz 0.70 -0.18 0.52 \
  --quat 0.31100 0.65408 0.40126 0.56075
```

规划器命令仍是：

```bash
python3 docs/scene_gento_skye.py --planner RRTConnect --time 1
```

脚本默认已经是 `RRTConnect`，`--time` 默认 8 秒。8 秒对这个规划器偏长。场景脚本现在不会读取上一行打印的关节角，它仍自己调用 `/compute_ik`。核对过的 7 个数要另外写成关节目标。

## 7 轴先交出一组站得住的关节角

一条手臂是 7 个转动关节，TCP 位姿是 6 维，同一个夹爪位姿对应一串关节解。腰在求解时锁住。默认左右臂都解，再留下更适用的那只，和 `scene_gento_skye.py` 里 `choose_arm()` 先两侧都问 KDL、再留行程更短的手臂是同一条规则。

MoveIt 的 KDL 插件从种子做局部迭代，失败后再随机换种子。随机解可以让末端靠近目标，肘却可能翻到身后，关节也可能贴着限位。这样的目标进了关节直线和 `RRTConnect` 之后，路会绕远，碰撞检查也更容易失败。求解器内部的循环见 [kdl_ompl.md](../tutorials/kdl_ompl.md)。

[kdl_arm_ik.py](../kdl_arm_ik.py) 用系统安装的 PyKDL。右臂链是 `waist_Link_3` 到 `TCP_Link_R`，左臂链是 `waist_Link_3` 到 `TCP_Link_L`，各自 7 个转动关节。每一组种子先交给 `ChainIkSolverPos_LMA`，6 个权重都是 1。解出的角越出 URDF 限位时，再用 `ChainIkSolverPos_NR_JL` 和 `ChainIkSolverVel_wdls` 在限位内重解。这样做的原因见 [ik_solving.md](ik_solving.md)。种子是该侧的参考角（`--seed`，不写就是 7 个 0；两侧当前角不同时用 `--seed-right` 和 `--seed-left`）、屈肘、几组 `Joint3` 滚转，再加上两组把手伸到身前的起始角。回零时两只手在身体两侧，单靠全 0 不容易够到桌面。conda 里的 Python 没有 PyKDL，用 `/usr/bin/python3`。

两侧都求出候选之后，按这个顺序留一只手：

1. 正运动学对得上的解才进入比较。对不上的一侧只留下残差。
2. 肘在身前的解优先于肘在身后的解。
3. 剩下的解里，取离该侧种子关节距离更小的一只。这就是 `choose_arm()` 的短行程。
4. 两侧距离差不超过 0.25 rad，而且目标的 y 离开中线超过 0.05 m 时，留目标那一侧的手臂：y < 0 用右臂，y > 0 用左臂。

上面那条命令没有写 `--arm`，腰锁在 0，两侧种子都是全 0。右臂对上了，左臂还差约 0.342 m 和 0.550 rad，所以留下右臂：

```text
selected=right_arm
reason=only_converged
right_arm converged=1 elbow_in_front=1 joint_distance=1.8977 limit_margin_rad=0.3116
left_arm converged=0 position_error_m=3.421e-01 rotation_error_rad=5.500e-01
```

右臂的 7 个角是：

```text
converged=1
elbow_in_front=1
position_error_m=6.506e-13
rotation_error_rad=0.000e+00
limit_margin_rad=0.3116
Joint1_R=-0.390595
Joint2_R=-0.994181
Joint3_R=-0.286879
Joint4_R=-1.266021
Joint5_R=-0.398655
Joint6_R=0.735633
Joint7_R=-0.274304
```

正运动学回到 `(0.700000, -0.180000, 0.520000)`。留下一组角，要同时满足：

- 位置残差小于 0.1 mm，姿态残差小于 0.0001 rad。
- 7 个角都在 URDF 限位里。`limit_margin_rad` 是离最近限位的距离。
- 肘在身前。规则与 `scene_gento_skye.py` 的 `elbow_in_front()` 相同。
- 这几条都通过时，取离 `--seed` 最近的一条。手上已经有关节角，就把那 7 个数传给 `--seed`，冗余解会留在当前姿态附近。上面这组解再作为种子求一次，关节距离是 0。

`converged=0` 时没有关节目标。脚本里右臂的 `CURRENT_ORIENTATION`（`0.70710678 0 0 0.70710678`）配合同一个坐标，右臂最近一次尝试还差 `8.531e-02` m 和 `3.257e-01` rad，退出码是 2。这个位姿不要送去规划。退出码 3 表示位姿对上了，但肘在身后，这次桌面抓取也不用。

腰已经升起或转开时，用 `--waist` 传入 `waist_jiont_1_prismatic`、`waist_joint_2`、`waist_joint_3`。这一步不读桌子和球。碰撞仍由后面的关节直线和 OMPL 做。`/usr/bin/python3 kdl_arm_ik.py --self-test` 用另一套 URDF 正运动学核对这条链，并做一次往返逆解。

打印出来的 7 个数都要写成 `JointConstraint`，再按下一节用 `RRTConnect`。`plan_gento_skye.py` 只约束了 `Joint1`、`Joint2`、`Joint4`，另外四个关节留在当前值，那不是这个 TCP 位姿的逆解。

## 这三次请求在做什么

`choose_arm()` 留下一条手臂，7 个关节。预抓取、抓取、撤退各来一次：

1. 关节目标来自上一节的 KDL 求解。脚本里的 `/compute_ik` 仍会自己求一次，肘在身后、或者构型已经撞上的解丢掉。逆解失败，这一段规划不会开始。
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
