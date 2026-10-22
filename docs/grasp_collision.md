# 桌面抓取的碰撞检查

这一套是机器人网格，不是 [`planning_from_scratch.md`](planning_from_scratch.md) 第 2 节里线段对矩形的精确判别。拼接顺序见 [grasp_trajectory_workflow.md](grasp_trajectory_workflow.md)。允许碰撞矩阵怎么生成，见 [srdf.md](srdf.md)。

`docs/scene_gento_skye.py` 不自己算两个网格相不相交。它把关节角交给 `move_group`：

- `/check_state_validity`：一个构型查一次。`start_is_valid()` 查回零后的起点，`goal_is_valid()` 查逆解和关节直线上的样本。请求里的 `group_name` 是 `both_arms`。
- `/plan_kinematic_path`：关节直线被挡住之后才走。OMPL 沿 `q(s) = (1-s) q_near + s q_new` 抽中间构型，每个构型再查一次。

一个构型无效，这条边就丢掉。中间构型多半不会变成 RViz 里的路点。`ompl_planning.yaml` 没有写 `longest_valid_segment_fraction` 或 `maximum_waypoint_distance`，仓库里因此说不出每隔多少弧度抽一个点。通过只说明抽到的那些构型没有碰到仍要查的对。

## 形状从哪来

手臂、夹爪、腰上仍保留 `<collision>` 的连杆，用的是 URDF 里的 STL，和显示网格通常是同一个文件。例如 `Link6_R` 的碰撞网格是 `meshes/Link6_R.STL`。仓库里没有把这些网格再收成一串碰撞球。

`generate_description.py` 删掉了 25 个连杆的 `<collision>`，`<visual>` 留着。它们在 RViz 里看得到，检查时没有形状：

`base_link`，`C1_Link` 到 `C8_Link`，`IMU_Link`，`XJ_Link`，`YQ_Link`，`YQL_Link`，`YH_Link`，`YHL_Link`，`YLD_Link`，`ZQ_Link`，`ZQL_Link`，`ZH_Link`，`ZHL_Link`，`ZLD_Link`，`head depth camera_Link`，`left eye camera_Link`，`right eye camera_Link`，`waist depth camera_Link`。

还带着碰撞网格的是这 31 个，全部是 `<mesh>`：

`Link1_R` … `Link8_R`，`TCP_Link_R`，`Link_Right_wrist`，`Arm_R9_Link`，`Arm_R10_Link`，`Arm_R11_Link`，左侧把 `_R` 换成 `_L`、`Right` 换成 `Left`，再加上 `waist_Link_1_prismatic`，`waist_Link_2`，`waist_Link_3`，`head_Link_1`，`head_Link_2`。

桌子、方块、球不来自 URDF。`apply_scene()` 放进 PlanningScene 的是盒子和球，坐标系 `base_link`：

| 物体 | 形状 |
|---|---|
| `table_top` | 盒子 1.50 × 2.40 × 0.05 m，中心 (1.25, 0, 0.335)，上表面 z = 0.36 |
| `table_legs` | 一个物体：灰色底板 1.62 × 2.52 × 0.03 m，中心 (1.25, 0, 0.295)，加四条桌腿 0.09 × 0.09 × 0.28 m |
| `cube` | 盒子，边长 0.06 m，中心 (0.70, 0, 0.39) |
| `ball_right` / `ball_left` | 球，中心 (0.45, ∓0.55, 0.86)，半径 0.07 m |
| `ball_high_right` / `ball_high_left` | 球，中心 (0.62, ∓0.28, 1.02)，半径 0.06 m |
| `ball_front` | 球，中心 (0.90, 0, 0.98)，半径 0.07 m |

## 一次检查怎么判

1. 用请求里的关节角做正运动学，把 31 块 STL 和场景里的盒子、球摆到世界上。夹爪开度是 `Arm_R10_Joint`、`Arm_R11_Joint`、`Arm_L10_Joint`、`Arm_L11_Joint`。这次抓取的三段手臂运动用 `FINGER_SPREAD = 0`。没写进请求的关节沿用场景里的当前值。
2. 两块都有形状、而且不在下面的跳过名单里，才算这一对。`move_group` 做几何相交。有重叠就记一条接触，名字是 `body1/body2`。
3. 有任何一条接触，`valid` 就是 false。

`elbow_in_front()` 不在这套检查里。它只用 `/compute_fk` 看肘的位置。`solve_ik()` 里 `avoid_collisions = false`，KDL 解位姿时不查这些网格。

夹爪从 0 插到 `FINGER_HOLD = 0.012` m、五段接成一条轨迹之后、以及回放时把方块移到两个 TCP 中点，都不再调用这次检查。

## 跳过的对

`gento_skye.srdf` 里有 126 行 `disable_collisions`，`reason` 全是 `Adjacent`。这个词只是标签。生成规则是每个连杆沿父连杆向上 3 级，见 `generate_description.py`。其中 39 对涉及已经没有碰撞网格的连杆，列在表里，检查时没有形状可算。

右臂和右夹爪：

| 连杆 | 跳过 |
|---|---|
| `Link1_R` | `Link2_R`，`Link3_R`，`Link4_R`，`waist_Link_1_prismatic`，`waist_Link_2`，`waist_Link_3` |
| `Link2_R` | `Link3_R`，`Link4_R`，`Link5_R`，`waist_Link_2`，`waist_Link_3` |
| `Link3_R` | `Link4_R`，`Link5_R`，`Link6_R`，`waist_Link_3` |
| `Link4_R` | `Link5_R`，`Link6_R`，`Link7_R` |
| `Link5_R` | `Link6_R`，`Link7_R`，`TCP_Link_R` |
| `Link6_R` | `Link7_R`，`Link8_R`，`TCP_Link_R` |
| `Link7_R` | `Link8_R`，`Link_Right_wrist`，`TCP_Link_R` |
| `Link8_R` | `Link_Right_wrist`，`TCP_Link_R` |
| `Link_Right_wrist` | `TCP_Link_R` |
| `Arm_R9_Link` | `Link7_R`，`Link8_R`，`TCP_Link_R` |
| `Arm_R10_Link` | `Arm_R9_Link`，`Link8_R`，`TCP_Link_R` |
| `Arm_R11_Link` | `Arm_R9_Link`，`Link8_R`，`TCP_Link_R` |

左臂把 `_R` 换成 `_L`，`Right` 换成 `Left`，一对不差。

腰和头：

| 连杆 | 跳过 |
|---|---|
| `base_link` | `waist_Link_1_prismatic`，`waist_Link_2`，`waist_Link_3` |
| `waist_Link_1_prismatic` | `waist_Link_2`，`waist_Link_3` |
| `waist_Link_2` | `waist_Link_3` |
| `waist depth camera_Link` | `waist_Link_1_prismatic`，`waist_Link_2`，`waist_Link_3` |
| `head_Link_1` | `head_Link_2`，`left eye camera_Link`，`right eye camera_Link`，`waist_Link_1_prismatic`，`waist_Link_2`，`waist_Link_3` |
| `head_Link_2` | `left eye camera_Link`，`right eye camera_Link`，`waist_Link_2`，`waist_Link_3` |
| `head depth camera_Link` | `head_Link_1`，`head_Link_2`，`waist_Link_3` |
| `left eye camera_Link` | `waist_Link_3` |
| `right eye camera_Link` | `waist_Link_3` |

底盘和轮子，对方大多已经没有碰撞网格：

| 连杆 | 跳过 |
|---|---|
| `C1_Link` … `C8_Link`、`IMU_Link`、`XJ_Link` | 各自只对 `base_link` |
| `YQ_Link`、`YH_Link`、`YLD_Link`、`ZQ_Link`、`ZH_Link`、`ZLD_Link` | `base_link` |
| `YQL_Link` | `YQ_Link`，`base_link` |
| `YHL_Link` | `YH_Link`，`base_link` |
| `ZQL_Link` | `ZQ_Link`，`base_link` |
| `ZHL_Link` | `ZH_Link`，`base_link` |

这些对关掉，是因为相邻几节的 STL 在关节附近叠在同一块空间里。`Link7_R` 和 `Link8_R` 中间还隔着 `TCP_Link_R`，零位时仍然重叠。不跳过的话，手臂还没动就被判成自己撞自己。

## 仍要查的对

31 块还有网格的连杆，两两成对，去掉上面已经跳过、并且双方都还有网格的那些，剩下 **378** 对自碰撞。再加每一块网格对 8 个场景物体。场景脚本没有把桌子、方块、球写进允许碰撞矩阵。

| 仍要查 | 对数 | 里面是什么 |
|---|---|---|
| 右臂内部 | 45 | 同一条右臂上隔得比 3 级更远的连杆，含夹爪。例如 `Link1_R` 对 `Link5_R` |
| 左臂内部 | 45 | 与右臂同一套 |
| 右臂对左臂 | 169 | 两只手、两条手臂之间。名单里没有跨左右的对 |
| 右臂对腰 | 33 | 右臂 11 块网格里，除了已经和腰跳过的，其余对 `waist_Link_1_prismatic`、`waist_Link_2`、`waist_Link_3` |
| 左臂对腰 | 33 | 与右边同一套 |
| 右臂对头 | 26 | 右臂、右夹爪对 `head_Link_1`、`head_Link_2`。头上的相机没有碰撞网格 |
| 左臂对头 | 26 | 与右边同一套 |
| 头对腰 | 1 | `head_Link_1` 对 `waist_Link_1_prismatic`。其余头—腰对已在跳过名单里 |
| 机器人对场景 | 31 × 8 | 上面 31 块网格，各自对 `table_top`、`table_legs`、`cube` 和五只球 |

`Link1_R` 那一行可以当成例子。它跳过 `Link2_R`、`Link3_R`、`Link4_R` 和三节腰。它仍然要查 `Link5_R` 以及更远的右臂连杆、整条左臂、两节头、桌板、桌腿、方块和五只球。
