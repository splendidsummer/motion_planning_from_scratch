# 一次规划里的两段循环

[返回目录](README.md)。在本文的桌面脚本中，笛卡尔位姿先交给 KDL 求逆解；脚本筛选关节解，再把关节目标交给 MoveIt 的 OMPL 规划管线。OMPL 在关节空间里找路，MoveIt 提供碰撞检查和时间参数化。**这个“先 IK、后关节目标规划”的分工适用于本脚本，不能推广成所有 MoveIt 位姿目标请求都只在规划前调用一次 IK。**

[桌面场景脚本](../docs/scene_gento_skye.py) 把这两段拆成 `/compute_ik` 和 `/plan_kinematic_path` 两类服务调用，中间还有 FK 和状态有效性检查；直线插值成功时不调用规划服务。[关节目标规划脚本](../docs/plan_gento_skye.py) 的目标已经是关节角，所以不走 IK。

以下参数以桌面脚本为准；KDL 实现按本机 ROS 2 Humble、MoveIt 2.5.9 核对，源码依据见文末。

## 调用顺序

桌面脚本里，一个抓取阶段是：

1. `solve_ik` 向 `/compute_ik` 要关节解。[逆运动学配置](../gento_skye_moveit_config/config/kinematics.yaml) 指定的求解器是 `kdl_kinematics_plugin/KDLKinematicsPlugin`。
2. `solve_ik` 用正运动学筛选肘的位置，并按与种子的关节距离排序。外层流程再用 `/check_state_validity` 检查候选构型，包括自碰撞和与场景物体的碰撞。
3. `plan_between` 优先尝试关节空间直线；检查失败或插值段数超过上限时，调用 `plan_group`，向 `/plan_kinematic_path` 发关节目标。`pipeline_id` 是 `ompl`，`planner_id` 是 `--planner` 选的名字，例如 `RRTConnect`。
4. 走 OMPL 管线时，[OMPL 配置](../gento_skye_moveit_config/config/ompl_planning.yaml) 里的 `AddTimeOptimalParameterization` 做时间参数化，请求里的速度、加速度缩放都是 0.25。直线分支的时间则由脚本自己分配，不经过这个适配器。

RViz 的普通 Plan 流程也要区分交互和规划：在 Humble 中，拖动末端标记时，交互组件先通过 IK 更新目标 `RobotState`；点击 Plan 后，界面调用 `setJointValueTarget()`，提交的是关节目标。不是必然在同一次 `plan()` 里先调用 `/compute_ik` 再规划；Cartesian Path 模式是另一条流程。

如果应用直接提交位置／姿态目标约束，MoveIt 的 OMPL 接口可使用 `IKConstraintSampler`，在规划期间调用 IK，采样多个目标关节状态。OMPL 核心算法不依赖 KDL，但不能因此说整个 MoveIt–OMPL 规划过程不会调用 IK。

<img src="figures/kdl-ompl-calls.svg" alt="位姿只进 /compute_ik。筛完肘和碰撞之后，交给 /plan_kinematic_path 的是关节角" style="max-width:100%; height:auto;">

在本脚本的规划请求里，位姿已经不在目标中。`goal_constraints` 的元素是 `Constraints` 消息，其中的 `joint_constraints` 才是一组 `JointConstraint`：关节名、关节角、上下容差。搜索状态也是关节角。这里不再需要 KDL 生成目标；KDL 的数值迭代本身不检查整条路，也不读取桌面和球的碰撞几何。

## KDL 里面的循环

左右臂的配置是：

```yaml
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
kinematics_solver_search_resolution: 0.005
kinematics_solver_timeout: 0.05
```

7 轴对完整的 6 维末端位姿通常有冗余：在雅可比满秩时，解在局部往往构成一维连续集合，而不只是“肘上、肘下”两个离散解。**7 轴不等于一定没有解析解**；能否推导解析解取决于机构和冗余参数化。这里选用的 KDL 插件采用数值迭代。

正运动学是 `ChainFkSolverPos_recursive`：连杆变换连乘，得到当前末端位姿。速度层是 `ChainIkSolverVelMimicSVD`：在当前关节角处，用加权 SVD 伪逆求解局部线性问题 $J(q)\,\Delta q \approx e$。这里完整位姿任务的 $J$ 是 $6\times7$，$e$ 是位置和旋转误差组成的 6 维向量；虽然接口把输出称为 `qdot`，位置迭代里把它用作关节增量，并不代表最终轨迹的执行速度。

SVD 默认阈值参数是 0.001，但该实现调用 Eigen 的 `setThreshold()`，用的是相对于最大奇异值的判定，不是简单丢弃所有绝对值小于 0.001 的奇异值。截断小奇异值方向可抑制伪逆在奇异位形附近的放大。若链中含 mimic 关节，其贡献会合并到驱动关节，不作为独立未知数；本项目手臂组的 7 个自由度不包含夹爪开合。

位置层是插件自己的 `KDLKinematicsPlugin::CartToJnt()`，**不是 `ChainIkSolverPos_NR_JL`**。从种子 $q$ 出发，其主要逻辑是：

1. 正运动学得到末端位姿，计算位置误差范数和旋转误差范数；两者均不大于 `epsilon` 时成功。
2. 通过速度层求关节增量；如果误差没有下降，则回退并缩小上一步的步长。
3. 用 `clipToJointLimits()` 裁剪增量，并降低触及限位的关节在后续求解中的权重；增量过小时，还可能施加小幅随机扰动。
4. 更新 $q \leftarrow q+\Delta q$，继续迭代；收敛、无法继续推进或达到迭代上限时退出。

该版本默认 `epsilon = 1e-5`、`max_solver_iterations = 500`，本项目配置没有覆盖它们。位置误差以米计，旋转误差以弧度计；这与规划目标的关节容差不是一回事。下图只展示主循环，回退和扰动细节以上述逻辑为准。

<img src="figures/kdl-newton.svg" alt="KDL 位置迭代示意：正运动学、加权 SVD、限位与步长处理；默认最多 500 次迭代，失败且未超时则换种子" style="max-width:100%; height:auto;">

一次局部求解只给出一个候选解，不保证它是离种子最近的解。插件先试请求提供的种子；失败或候选被有效性回调拒绝后，再随机换种子，直到成功或超时。该版本的随机重启循环没有按 `kinematics_solver_search_resolution: 0.005` 枚举冗余关节，不能把这个配置值解释为实际使用的扫描步长。它也不以肘朝向或避障作为零空间优化目标。

桌面脚本把每次 `/compute_ik` 请求的超时改成 80 ms；`lock_orientation=True` 时是 250 ms。该开关控制脚本是否只尝试指定姿态，而不是把 KDL 从“仅位置 IK”切换成“完整位姿 IK”：这里每次请求本来就包含位置和姿态。插件在局部求解尝试之间检查超时，因此这个值不是严格的服务墙钟时限；脚本还会为不同位置／姿态发出多次请求。

脚本把 `avoid_collisions` 设为 `False`，将碰撞筛选移到外层。若设为 `True`，MoveIt 可通过解有效性回调拒绝碰撞候选，KDL 再继续搜索，而不是每一步数值迭代都检查碰撞。短超时内若只找到碰撞解，启用检查就可能找不到可接受解；这并非必然。实际顺序是：`solve_ik` 用 `elbow_in_front` 和 `/compute_fk` 筛选肘的位置，再按关节距离排序、截取候选；外层单臂或双臂配对流程才调用 `goal_is_valid`，通过 `/check_state_validity` 检查组合后的构型。IK 返回成功本身不代表无碰撞。

<img src="figures/kdl-seeds.svg" alt="同一个夹爪位姿：肘在身前的解留下，肘在身后的丢掉，连杆进桌面的丢掉" style="max-width:100%; height:auto;">

## OMPL 里面的循环

本脚本交给 OMPL 管线的是筛选后的关节目标。`plan_group` 把每个目标关节写成一条 `JointConstraint`，容差 ±0.05 rad。到达目标容差范围不等于精确到达原 IK 位姿，也不约束末端沿途保持姿态或走笛卡尔直线。规划组 `right_arm` / `left_arm` 是 7 维，`both_arms` 是 14 维。夹爪开度写在起始状态里，不参与搜索；夹爪碰撞几何仍随手臂移动，按场景的碰撞配置参与检查。

注意：[关节目标规划脚本](../docs/plan_gento_skye.py) 使用的是 ±0.08 rad 容差、4 次规划尝试，不是桌面脚本的 ±0.05 rad、8 次。

OMPL 的循环由 `--planner` 决定，共同的部分是：

1. 起点来自请求的 `start_state`，与当前场景状态合并；配置中的起点修正适配器还可能修正轻微越界或碰撞。终点是满足目标关节容差的有效构型。
2. 在关节空间里采样和连接状态，通过 MoveIt 提供的状态有效性检查做离散碰撞检测；具体策略由算法决定，并非所有算法都长树。
3. 单个 `RRTConnect` 实例通常找到可行解就结束搜索；`RRTstar` 等优化型规划器通常会在剩余预算内继续改善路径，也可能因其他终止条件提前结束。管线之后还可简化和插值路径，因此“找到第一条路”不等于服务立刻返回。各算法的循环见同目录的单篇，同一类树的改法集中在 [KPIECE、BKPIECE、LBKPIECE、SBL、EST、TRRT、BiTRRT、AnytimePathShortening、FITstar](KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md)。
4. 场景脚本的 `num_planning_attempts = 8` 表示请求多次求解。Humble 的 MoveIt OMPL 接口可通过 `ParallelPlan` 并行或分批运行多个规划器实例，并进行路径混合；它不是“失败后才重开一棵树”的简单重试，也不是每次都获得完整的 `allowed_planning_time`。这些尝试共享本次请求的时间预算。

`plan_between` 先调用客户端的 `interpolated_motion`，并不是 OMPL 内部先试一条直线。令 `peak` 为各关节起终点差的最大绝对值，段数为 `max(4, ceil(max(peak, 1e-4) / 0.08))`：每个关节相邻采样点的变化不超过约 0.08 rad，不是整组关节的欧氏距离为 0.08 rad。段数超过 50 时直接跳过，否则对包含起终点的所有采样点调用 `/check_state_validity`。全部通过就直接生成轨迹；检查失败或超过段数上限，才转到 `plan_group`。

这条捷径旨在减少短程抓取阶段不必要的绕行，但“末端只差几厘米”不保证关节距离很短；OMPL 也不会有意接受检测到的碰撞。两条分支的离散检查都不能证明采样点之间连续无碰撞。

<img src="figures/ompl-joint-goal.svg" alt="客户端直线的离散检查通过后由脚本写时间；检查失败或段数过多则交给 OMPL 管线搜索，再由 TOTG 做时间参数化" style="max-width:100%; height:auto;">

## 时间打在路径之后

OMPL 的几何规划结果本身没有执行时间。MoveIt 的 `AddTimeOptimalParameterization` 使用 TOTG，依据 [关节速度和加速度上限](../gento_skye_moveit_config/config/joint_limits.yaml) 及请求中的两个 0.25 缩放系数，生成 `time_from_start`、速度和加速度。因此 `/plan_kinematic_path` 的成功响应已经经过时间参数化，而不是还要由客户端给 OMPL 结果补时间。

TOTG 不只是给原折线的每个点贴时间戳：它会拟合并重采样路径，允许在路径容差内偏离原路径。TOTG 自身不做碰撞检查；Humble 的规划管线默认会在适配器执行后复查输出航点，但这仍不是连续碰撞保证。

直线分支不调用 TOTG。脚本用 $T=\max(\mathrm{peak}/(0.25\pi),\,0.6)$ 秒作为总时长，均匀分配给插值点，只填写位置和时间；它没有读取逐关节加速度上限，也没有生成速度／加速度字段。不能把这条分支称为满足相同限速、限加速度约束的 TOTG 轨迹。

IK 搜索时间、路径搜索时间和轨迹执行时长是三个不同概念：

| 参数 | 谁在用 | 桌面脚本里的值 |
|---|---|---|
| `/compute_ik` 的 `timeout` | 单次请求中 KDL 的局部求解与重启 | 80 ms，锁姿态时 250 ms；不是整个 `solve_ik` 的总预算 |
| `kinematics_solver_timeout` | 未显式指定超时时使用的默认逆解预算 | 0.05 s；脚本在请求中显式指定了超时 |
| `allowed_planning_time` | OMPL 接口搜索及后续路径简化的预算 | `--time`，默认 8 s；不包含前面的 IK 和客户端直线检查 |
| `time_from_start` | 轨迹点相对于轨迹起点的执行时间 | 由 TOTG 或直线分支的手工时间分配得到，不等于规划耗时 |

在这条脚本流程中，IK 或候选筛选失败时，相应候选不会进入后续路径规划。规划服务失败则不一定是“树在障碍里没接上”：起点／目标无效、请求或规划器配置错误、搜索超时、时间参数化失败、最终路径复查失败等都可能导致失败，应结合 `error_code` 和服务端日志定位。

## 实现依据

- [MoveIt 2.5.9：KDL 插件](https://github.com/moveit/moveit2/blob/2.5.9/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp)：`CartToJnt()`、默认迭代参数、随机重启和限位处理。
- [MoveIt 2.5.9：速度层 SVD](https://github.com/moveit/moveit2/blob/2.5.9/moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp)：加权雅可比、mimic 映射和 Eigen 阈值接口。
- [MoveIt Humble：RViz 规划入口](https://github.com/moveit/moveit2/blob/humble/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame_planning.cpp)与[目标约束采样](https://github.com/moveit/moveit2/blob/humble/moveit_planners/ompl/ompl_interface/src/detail/constrained_goal_sampler.cpp)：关节目标与位姿目标的不同路径。
- [MoveIt Humble：OMPL 规划上下文](https://github.com/moveit/moveit2/blob/humble/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp)：多次求解、时间预算和路径后处理。
- [MoveIt Humble：TOTG](https://github.com/moveit/moveit2/blob/humble/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp)与[规划管线](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp)：时间参数化与输出路径检查。Humble 分支链接会随维护更新。
