# Gento Skye：关节状态循环、MoveIt 规划、RViz 显示

模型来自同目录的 `Gento Skye urdf(gripper)/Gento Skye urdf/urdf/Gento Skye urdf.urdf`。MoveIt 配置在 `gento_skye_moveit_config/`。环境是 Ubuntu 22.04 上的 ROS 2 Humble。

新开的终端已经在 `~/.bashrc` 里加载了 Humble 和 `~/.local/opt/ros-humble-user`。一条命令同时打开关节状态发布、`move_group` 和 RViz：

```bash
ros2 launch gento_skye_moveit_config demo.launch.py
```

这条 launch 会拉起：

| 进程 | 作用 |
|---|---|
| `joint_state_publisher` | 按 URDF 发布关节位置，RViz 里的实体模型跟着动 |
| `robot_state_publisher` | 把关节位置变成各连杆的 TF |
| `move_group` | MoveIt 规划。规划管线是 OMPL，每次规划再选具体算法 |
| `rviz2` | 读 `gento_skye_moveit_config/launch/moveit.rviz` |

窗口标题是 `moveit.rviz - RViz`。不要再开第二份，两份会抢同一个节点名。

已经开着、还没加载 ROS 的终端先执行：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/.local/opt/ros-humble-user/local_setup.bash"
```

停掉当前这份：

```bash
pkill -INT -f 'ros2 launch gento_skye_moveit_config'
```

## 在 RViz 里规划

左侧 **MotionPlanning** 面板：

1. **Planning Group** 选一个组。
2. 在三维视图里拖橙色目标标记。绿色是起始姿态。
3. 点 **Plan**。半透明手臂是规划出的轨迹。

| 组 | 关节 |
|---|---|
| `right_arm` | `Joint1_R` … `Joint7_R`，末端是 `TCP_Link_R` |
| `left_arm` | `Joint1_L` … `Joint7_L`，末端是 `TCP_Link_L` |
| `both_arms` | 上面 14 个关节一起规划。这个组没有笛卡尔逆解，在 **Joints** 里改两侧目标角，再点 **Plan** |
| `right_gripper` | `Arm_R10_Joint`，`Arm_R11_Joint` |
| `left_gripper` | `Arm_L10_Joint`，`Arm_L11_Joint` |
| `waist` | `waist_jiont_1_prismatic`，`waist_joint_2`，`waist_joint_3` |
| `head` | `head_joint_1`，`head_joint_2` |

**Plan** 只在这台机器上算出几何路径。这里没有接真机控制器，**Execute** 不会驱动机器人。

## 规划代码和算法

手臂规划的代码是 `plan_gento_skye.py`。它向已经启动的 `move_group` 发一次规划请求，把结果打在终端上，并把轨迹发到 RViz 的 `/display_planned_path`。二维教学脚本 `planners_2d.py` 不控制这台机器人。

先保持上面的 `demo.launch.py` 开着，再开一个终端换算法：

```bash
cd ~/Projects/motion_planning_from_scratch
./plan_gento_skye.py --group both_arms --planner RRTConnect
./plan_gento_skye.py --group right_arm --planner RRTstar --time 5
./plan_gento_skye.py --group left_arm --planner PRM
```

`--group` 用 `right_arm`、`left_arm` 或 `both_arms`。`--planner` 就是算法名。`--time` 是这次规划最多用的秒数，默认 5。`RRTConnect` 找到一条能走的路就停。`RRTstar` 会把 `--time` 用来把路径磨短，所以给它的时间要比 `RRTConnect` 长。`PRM` 先建路网再查询。

算法名写在 `plan_gento_skye.py` 的 `PLANNERS` 里，命令行和 RViz 下拉框用同一组名字：

```python
PLANNERS = (
    "RRTConnect",
    "RRT",
    "RRTstar",
    "PRM",
    "PRMstar",
    "EST",
    "SBL",
    "KPIECE",
    "BKPIECE",
    "LBKPIECE",
    "TRRT",
    "BiTRRT",
    "AnytimePathShortening",
)
```

选算法的参数是 `--planner`，默认 `RRTConnect`：

```python
parser.add_argument("--group", choices=tuple(GOALS), default="both_arms")
parser.add_argument("--planner", choices=PLANNERS, default="RRTConnect")
parser.add_argument("--time", type=float, default=5.0, help="seconds the planner may run")
```

选中的名字写进规划请求的 `planner_id`，管线固定是 `ompl`：

```python
plan.group_name = group
plan.planner_id = planner
plan.pipeline_id = "ompl"
plan.num_planning_attempts = 4
plan.allowed_planning_time = allowed_time
```

目标关节角在同一个文件的 `GOALS` 里。没写进目标的关节保持当前角度。`both_arms` 没有笛卡尔逆解，双臂目标就是两侧关节角：

```python
GOALS = {
    "right_arm": {"Joint1_R": 0.6, "Joint2_R": 0.35, "Joint4_R": -0.9},
    "left_arm": {"Joint1_L": -0.6, "Joint2_L": -0.35, "Joint4_L": -0.9},
    "both_arms": {
        "Joint1_R": 0.6,
        "Joint2_R": 0.35,
        "Joint4_R": -0.9,
        "Joint1_L": -0.6,
        "Joint2_L": -0.35,
        "Joint4_L": -0.9,
    },
}
```

在 RViz 里换算法不改代码。打开 **MotionPlanning** 面板的 **Context** 页，**Planning Library** 保持 `ompl`，下面的下拉框选和 `--planner` 相同的名字，再点 **Plan**。

五个主规划器各有一篇，文件名和 `--planner` 相同。KPIECE 一族、SBL、EST、TRRT、BiTRRT、AnytimePathShortening 写在 `tutorials/KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md`。目录是 `tutorials/README.md`。

| 算法 | 什么时候用 |
|---|---|
| `RRTConnect` | 先找一条能走的路。双臂点到点用这个 |
| `RRT` | 只从起点长一棵树 |
| `RRTstar` | 在 `--time` 里继续把路径变短 |
| `PRM` | 先抽样一张路网，再在网上查询 |
| `PRMstar` | 近邻个数随点数增加，路径通常更短 |
| `EST` | 按稀疏程度挑节点，再在它附近采样 |
| `SBL` | 两棵树，边先假定能走，成为候选路径再检查 |
| `KPIECE` | 在投影格子上优先扩张边界 |
| `BKPIECE` | 两棵 KPIECE 树 |
| `LBKPIECE` | 两棵 KPIECE 树，碰撞检查推迟到候选路径 |
| `TRRT` | 少收短边；有随构型变化的代价时才避开高代价区域 |
| `BiTRRT` | 两棵 TRRT 树 |
| `AnytimePathShortening` | 反复跑内层规划器，再用剩下的时间缩短、拼接 |

成功时终端会打印 `error_code=1`、轨迹点数、关节名、时长和末点关节角。`error_code` 不是 1 就是没规划出来。

## 模型上的两处处理

SolidWorks 导出的 `Joint1_L` 上下限都是 0，左肩转不了。加载时用了同目录 `Pro-m6-696` 里这个关节的范围 `±3.1067`，和右肩一致。原始 URDF 没有改。

底座、外壳和相机网格只用于显示，没有放进碰撞体。手腕附近几节连杆在零位时网格叠在一起，相邻三节的碰撞检查被关掉了，否则一上来就被判成碰撞，规划无法开始。
