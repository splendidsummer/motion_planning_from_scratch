# Gento Skye 的 SRDF

SRDF（Semantic Robot Description Format）是 MoveIt 在 URDF 之外读取的语义文件。本项目的文件是 [gento_skye.srdf](../gento_skye_moveit_config/config/gento_skye.srdf)。启动时 [demo.launch.py](../gento_skye_moveit_config/launch/demo.launch.py) 把它作为 `robot_description_semantic` 交给 `move_group`。

它回答三件事：哪些关节作为一组一起规划，底座相对世界怎么固定，以及哪些连杆对可以跳过碰撞检查。关节限位、速度、网格形状在 [URDF](../gento_skye_moveit_config/config/gento_skye.urdf) 里；速度和加速度的 YAML 覆盖见 [关节限制说明](joint_limits.md)。

这份 SRDF 由 [generate_description.py](../gento_skye_moveit_config/generate_description.py) 从 URDF 生成。手改 SRDF 后如果重新运行该脚本，生成规则会覆盖手改内容。

## 1. 底座如何固定

```xml
<virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base_link"/>
```

`virtual_joint` 不存在于 URDF。它把 `base_link` 固定在 `world` 上，所以规划时底盘不平移、不转动。轮组关节 `ZQ`、`ZQL`、`YQ`、`YQL`、`ZH`、`ZHL`、`YH`、`YHL` 没有被放进任何规划组，桌面 RRT 不会去搜索它们。

## 2. 哪些关节一起动

一个 `<group>` 是一次规划请求可以主动改变的关节集合。请求 `group_name` 时，组内关节是搜索变量，组外关节保持当前状态。

| 规划组 | SRDF 里怎么写 | 实际一起动的关节 |
|---|---|---|
| `right_arm` | 从 `waist_Link_3` 到 `TCP_Link_R` 的链 | `Joint1_R` … `Joint7_R` |
| `left_arm` | 从 `waist_Link_3` 到 `TCP_Link_L` 的链 | `Joint1_L` … `Joint7_L` |
| `both_arms` | 包含上面两个组 | 左右臂共 14 个关节 |
| `right_gripper` | 列出两个关节 | `Arm_R10_Joint`、`Arm_R11_Joint` |
| `left_gripper` | 列出两个关节 | `Arm_L10_Joint`、`Arm_L11_Joint` |
| `waist` | 列出三个关节 | `waist_jiont_1_prismatic`、`waist_joint_2`、`waist_joint_3` |
| `head` | 列出两个关节 | `head_joint_1`、`head_joint_2` |

手臂用的是 `<chain>`，不是把七个关节一个个写出来。MoveIt 沿 URDF 从链的起点连杆走到终点连杆，把路径上的活动关节放进组。链的起点连杆自身的父关节不算在内，所以 `waist_Link_3` 的升降和腰部旋转不进入 `right_arm` 或 `left_arm`。`TCP_Link_R` 由固定关节 `JointTCP_R` 接在 `Link7_R` 上，固定关节没有位置变量，因此这条链正好是 7 个旋转关节。

夹爪接在 TCP 之后：`TCP_Link` → 固定关节 → `Link8` → `Arm_*9_Link` → 两个指爪。它们不在手臂链的起点和终点之间，所以不在 `both_arms` 的 14 个搜索变量里。指爪网格仍装在手臂上，手臂一动，夹爪几何会跟着动，并用当前的开合宽度参加碰撞检查。`Arm_R11_Joint` 和 `Arm_L11_Joint` 在 URDF 里 mimic 另一指。

桌面脚本的预抓取、抓取和撤退都请求 `both_arms`。同一次 RRT 同时搜索 14 个手臂关节。腰、头、轮组和夹爪开合不由这次搜索改变。夹爪开度是脚本在各阶段事先写成 0 或 0.04 m 的。头部左右转和点头属于 `head` 组，只有单独请求该组时才会被规划。

两个末端执行器条目把夹爪组安到腕部连杆上：

```xml
<end_effector name="right_gripper" parent_link="Link8_R" group="right_gripper"/>
<end_effector name="left_gripper" parent_link="Link8_L" group="left_gripper"/>
```

这是给 MoveIt 的末端名称，不增加新的运动关节。

## 3. 哪些零件之间不做碰撞检查

SRDF 后部每一行

```xml
<disable_collisions link1="Link7_R" link2="Link8_R" reason="Adjacent"/>
```

都是允许碰撞矩阵里的一对连杆。规划时这一对不再互相检查。`reason="Adjacent"` 只是生成时写上的标签。

生成规则是：对每个连杆，沿父连杆向上走 3 级，把该连杆和这三级祖先都配成一对。因此被跳过的不只是直接相连的两节，还包括隔一级、隔两级的连杆。腕部网格和再往上两节的网格经常重叠，只跳过直接父子仍会把 `Link7` 和 `Link8` 报成碰撞，所以这里走了 3 级。

被跳过的典型部分：

- 手臂上相邻或隔得很近的连杆，例如 `Link6_R` 与 `Link7_R`、`Link7_R` 与 `TCP_Link_R`。
- 腕部和夹爪附近的连杆，例如 `Arm_R10_Link` 与 `Link8_R`、`Arm_R10_Link` 与 `TCP_Link_R`。
- 头壳与眼睛相机、头壳与 `waist_Link_3`。
- 腰部三节之间，以及它们和 `base_link`。
- 轮组和底盘外壳之间。这些外壳的碰撞网格在生成 URDF 时还被另行去掉，SRDF 这一行是第二层处理。

桌子、桌腿、方块、障碍球不在这个名单里。它们由场景脚本放进 PlanningScene，手臂和夹爪仍要与它们检查碰撞。左右臂之间、手臂和头部之间，只要某一对没有出现在 `disable_collisions` 里，就仍然检查。

因此“规划状态有效”表示：在当前关节值和场景下，没有碰到仍被检查的连杆对和场景物体。它不表示所有网格两两都量过距离。

## 4. 允许碰撞矩阵是什么，这里怎么构成

允许碰撞矩阵（Allowed Collision Matrix，ACM）是一张“谁和谁可以跳过碰撞检查”的表。一行一列都是名字，可以是机器人连杆，也可以是场景里的物体。某一格为允许时，MoveIt 不再计算这一对的距离；格子空着或为不允许时，这一对只要双方都有碰撞网格，就会被检查。

夹爪开度不写在这张表里。预抓取、抓取、撤退把指爪设成 0 或 0.04 m，那是机器人状态。MoveIt 先用这个状态摆好手臂和夹爪的网格，再用 ACM 决定哪些网格对可以不查。

本项目没有在场景脚本里调用 ACM 接口。矩阵在 `move_group` 加载 SRDF 时构成，来源就是上一节的 `disable_collisions`：

1. [generate_description.py](../gento_skye_moveit_config/generate_description.py) 对每个连杆向上找 3 级父连杆，写成 `disable_collisions`，`reason` 一律标成 `Adjacent`。
2. 启动时 [demo.launch.py](../gento_skye_moveit_config/launch/demo.launch.py) 把 `gento_skye.srdf` 作为 `robot_description_semantic` 载入。
3. MoveIt 为其中每一对连杆在 ACM 里记一条“允许碰撞”。
4. [scene_gento_skye.py](scene_gento_skye.py) 只向 PlanningScene 添加、移动或删除 `table_top`、`table_legs`、`cube` 和各球。它不给这些物体写 ACM 条目，所以它们和仍有碰撞网格的机器人连杆都要检查。

还有一层发生在矩阵之前。生成 URDF 时，`base_link`、轮组、`C1_Link` 到 `C8_Link`、`IMU_Link`、`XJ_Link` 的碰撞网格被去掉。没有碰撞网格的连杆不会产生接触，即使 ACM 里没有它们。

所以一次 `both_arms` 检查实际是：

- 14 个手臂关节用 RRT 改变；腰、头、轮组保持当前角度。
- 夹爪网格用该阶段固定的开度，随手臂摆到新位置。
- 自碰撞只跳过 ACM 里那 3 级近邻。
- 桌板、底板、桌腿、方块和球与手臂、夹爪的碰撞照常检查。

## 5. 和桌面 RRT 的关系

[RRT 教程](../tutorials/RRT.md) 里的 14 维构型来自 `both_arms`。一次扩展改变的是这 14 个关节；碰撞检查使用 URDF 网格、上面这份允许碰撞矩阵，以及已经提交的桌面物体。把某对机器人连杆放进 `disable_collisions`，只影响机器人自碰撞。要忽略夹爪和桌子，必须改场景或另写允许碰撞，不能靠现有的 `Adjacent` 列表。
