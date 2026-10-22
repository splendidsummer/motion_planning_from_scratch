# RRTConnect

[返回目录](README.md)。二维教学代码在 [`planning_from_scratch.md`](../docs/planning_from_scratch.md) 第 5 节，函数是 `rrt_connect`。下面这套步骤跑在 Gento Skye 的关节空间里。场景、沿途检查和步长估算见 [RRT](RRT.md)。

[桌面场景脚本](../docs/scene_gento_skye.py) 先为左右手各求一次从上往下的抓取，留下关节距离 home 更近的那条手臂。规划组是 `right_arm` 或 `left_arm`，构型 q 是 `Joint1` 到 `Joint7` 这 7 个关节角。另一条手臂停在 0。夹爪开度写在起始状态里，不参与搜索；夹爪网格仍随手臂运动，参与和桌板、方块、球的碰撞。目标是 KDL 给出的那组关节角，每个关节容差 ±0.05 rad。`plan_gento_skye.py` 也可以选 `both_arms`，那时 q 是 14 维。

`plan_between` 先试关节空间直线。直线无碰就直接当轨迹，下面的两棵树不会启动。从 home 到预抓取经常被挡住，才会进入 RRTConnect。

RRTConnect（Kuffner、LaValle，2000）维护两棵树。T_a 从起始关节角长，T_b 从目标容差里的一个构型长。每一轮：

1. 在关节限位盒子里抽一个 q_rand，对 T_a 做一次 `EXTEND`。
2. 如果没有 Trapped，就对 T_b 做 `CONNECT`：朝刚刚得到的 q_new 反复 `EXTEND`，直到 Reached 或 Trapped。
3. 交换两棵树，下一轮从另一边长。

距离是 7 个关节角的欧氏长度。最近的节点是关节角最接近的那个构型。正运动学之后，这个构型的 TCP 可以离方块更远，肘却更靠近球。关节空间里的一段直线，到了桌面上可以是指尖先下降再抬起。

## 一轮在手臂上怎么长

一次 `EXTEND`：在要长的那棵树里找离目标最近的关节构型，朝目标走出不超过 η 的一段。这段通过碰撞检查，才把终点加进树。走到目标是 Reached，停在半路是 Advanced，线段碰障是 Trapped，终点不加入。

第一轮 T_a 只有 home（七个关节为 0）时，q_near 就是 home。朝 q_rand 走出的那一步，检查的是整条手臂：指尖进入桌板、前臂撞上球、手臂扫到另一侧，都是 Trapped。无碰才得到 q_new。

`CONNECT` 的目标固定成另一棵树刚收下的 q_new，不朝 q_rand 长。T_b 的根在预抓取或抓取逆解的 ±0.05 rad 里。里面还是同一个 `EXTEND`：

1. 在 T_b 里找离这个 q_new 最近的关节构型。这是这一步的 q_near。
2. 从 q_near 朝 q_new 走出一段，长度不超过 η。这段无碰撞，才把终点加进 T_b。
3. 终点就是 q_new：返回 Reached。两棵树在这个关节构型接上，`CONNECT` 停止。
4. 终点还在半路：返回 Advanced。下一次 `EXTEND` 重新找最近节点。沿着这条关节直线走时，最近的就是刚加进去的点，所以下一段从它接着走。
5. 这一段碰到桌板、球或自身：返回 Trapped。终点不加入 T_b，`CONNECT` 停止，这一轮没有接上。

`CONNECT` 会把步长 η 用满，所以中间常常是一条关节空间直线。手从上方落到方块旁边时，这条直线会反复顶在球或桌沿上，Trapped 把省下的时间还回去。

然后交换。节点数量不变，机器人也不动。下一轮原来的 T_b 先朝一个新的 q_rand 走一步，原来的 T_a 再朝那个新构型一步一步接。因为每一轮都交换，要把「哪一棵树的根是 home」记下来。`rrt_connect` 用 `a_is_start` 记这件事。

两棵树接上时，路径是

<img src="equations/RRTConnect-1.svg" alt="起点到汇合点再到终点" style="max-width:100%; height:auto;">

汇合点在两棵树里各有一份相同的关节角，拼接时去掉一份。

## 什么时候用

它和 RRT 一样，接到一条能走的关节路径就停，不继续把路径磨短。空旷时每棵树大约负责一半关节路程，所以通常比只从 home 长的树更早返回。`--planner RRTConnect` 是桌面脚本和 `plan_gento_skye.py` 的默认规划器。`--time` 给 1 秒量级通常够用；脚本里 `num_planning_attempts = 8`，一次没接上会重开两棵树。

要更短的关节路径时换 [RRTstar](RRTstar.md) 或 [AnytimePathShortening](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#anytimepathshortening)，并把时间加长。

[OMPL 配置](../gento_skye_moveit_config/config/ompl_planning.yaml) 没有写 `range`。头文件里的默认比例是关节限位盒子对角线的 0.2。单臂对角线约 12.67，对应 η ≈ 2.53。这是估算，不是从正在运行的 `move_group` 读出的值。2.53 是 7 个关节角变化量的合成长度。
