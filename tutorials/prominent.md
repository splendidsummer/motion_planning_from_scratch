# 值得掌握的五个

[返回目录](README.md)。每个名字的循环在同目录的单篇里。取舍见 [pros_cons.md](pros_cons.md)，第一条路和长度见 [performance.md](performance.md)，检测和近邻见 [efficiency.md](efficiency.md)。

`--planner` 列表里的规划器大多是同一个循环的改法。值得掌握的是 RRT、RRTConnect、RRT*、PRM 和 BIT*。

## 这个循环

[RRT](RRT.md) 的一轮做四件事：在关节限位里抽一个构型，在已有的树上找最近节点，朝它走出不超过步长 η 的一段，这段无碰才把新点写入树。树从起点长。接到落在目标容差里的构型就停。

后面四个改的是这个循环里的一件事：

- [RRTConnect](RRTConnect.md) 维护两棵树。一轮里一棵只走一步，另一棵朝刚得到的新点一直走，然后交换。
- [RRT*](RRTstar.md) 仍是从起点长的一棵树。父节点改成邻域里「从起点走到新点」总代价最小的那个；写进树之后，附近已有的点若改道经过它更短，就换成它当父节点。这就是 rewire。
- [PRM](PRM.md) 把树换成一张无向路网。建图时不知道这一次的起终点。查询才把两个点接上去，在网上做最短路。
- [BIT*](BITstar.md) 每轮加一批样本，看成一张还没做碰撞检测的路网，只检查有希望改短当前解的边。有解之后，新样本收进以起终点为焦点、长轴为当前最好代价的椭球。

## 为什么留这五个

| 规划器 | 为什么留它 |
|---|---|
| [RRT](RRT.md) | 最基本的树。采样、找最近邻、走一步、做一次碰撞检测。其余算法都是在改这个循环。 |
| [RRTConnect](RRTConnect.md) | 两棵树，一棵从起点长，一棵从终点长。想尽快得到一条能走的路时用它。MoveIt 在关节空间点目标上通常的默认规划器，也是本仓库双臂点到点的默认选择。 |
| [RRTstar](RRTstar.md) | 同一棵树，多了按代价选父节点和 rewire。可以多等几秒、希望路径更短时用它。 |
| [PRM](PRM.md) | 与某一对起终点无关的路网。同一场景要查询很多次时用它。[PRM*](PRMstar.md) 是同一张图，近邻个数随点数增加，路径可以趋近最短。 |
| [BITstar](BITstar.md) | 现在常用的渐近最优规划器。按批搜索一张隐式路网，并把新样本收进「比当前最好路径更短」的椭球里。实际缩短路径通常比 RRT* 更快。 |

点到点先用 `RRTConnect`，`--time` 给 1 秒量级通常够用。可以等数秒、希望关节折线更短时用 `RRTstar`。`BITstar` 和 `FITstar` 不在本仓库的 `--planner` 列表里，脚本和 RViz 下拉框选不到。Humble 自带的 OMPL 1.7 有 `BITstar`。`FITstar` 的写法在 [FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#fitstar)。

同一个 `--time` 里，等到返回的时间通常是 RRT\* ≥ RRT ≥ RRTConnect。

RRT 和 RRTConnect 接到第一条能走的关节路径就停。RRT\* 找到之后继续 rewire，把预算用完。两边都成功时，RRT\* 的等待时间就是 `--time`。两边都没找到时，两者都停在预算上，这时相等。RRT\* 的第一条路也更晚：RRT 每个新点只查一条边，RRT\* 要看近邻。单臂 n = 1000 时默认 k = 24。

RRT 通常比 RRTConnect 久。RRTConnect 从起点和目标容差各长一棵树，空旷时每棵大约只走一半关节路程。手往方块下落、反复顶到桌沿或球时，失败的连接步都做了碰撞检查，RRTConnect 可以不比单向 RRT 快。

关节路径的长短不跟着这条顺序。RRT 和 RRTConnect 都停在第一条可行路上。RRT\* 多花的时间用来把关节折线磨短。

`PRM` 的多查询要路网留得下来。MoveIt 一次 `plan()` 结束就把图丢掉，下一次请求不继承。同一次调用里回答很多个起终点，这个好处才看得到。

## 其余名字

下面这些是 OMPL 对同一类树的专门改法，写在 [KPIECE、BKPIECE、LBKPIECE、SBL、EST、TRRT、BiTRRT、AnytimePathShortening、FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md)。

| 名字 | 改的是哪一步 |
|---|---|
| [KPIECE](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#kpiece)、[BKPIECE](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#bkpiece)、[LBKPIECE](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#lbkpiece)、[SBL](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#sbl) | 用低维投影决定往哪扩，或把碰撞检查推迟到候选路径上。碰撞检测很贵时有用。投影把通道压扁时，树会去扩别处。 |
| [EST](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#est) | 较早的单棵树，优先往稀疏区域扩。点到点通常比 RRTConnect 慢。 |
| [TRRT](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#trrt)、[BiTRRT](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#bitrrt) | 拒绝走进高代价区域。这张桌子和球的场景没有这种代价，额外的检验几乎不起作用。 |
| [AnytimePathShortening](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#anytimepathshortening) | 包在别的规划器外面、持续把路径缩短的一层。搜索由内层规划器做，默认内层是 RRTConnect。 |
| [FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md#fitstar) | 知情树的一种变体：批次随当前解的椭球变小，另有一棵从终点长回来的启发式树。文献里作为这一类被引用的是 BIT*。 |

读的顺序是 RRT、RRTConnect、RRT*、PRM（含 PRM*）、BIT*。投影、推迟检查和代价场，用到那种场景再看 [KPIECE、BKPIECE、LBKPIECE、SBL、EST、TRRT、BiTRRT、AnytimePathShortening、FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md)。
