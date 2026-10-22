# 为什么现在算不出关节加速度

本文说明：仓库里已有的力矩不能换成 [joint_limits.yaml](../gento_skye_moveit_config/config/joint_limits.yaml) 的 `max_acceleration`。关节速度、控制器百分比和夹爪协议的完整对照见 [关节限制说明](joint_limits.md)。

> **结论：** 按当前仓库里的数据，算不出可以写入 `max_acceleration` 的加速度。108、66、18、194、22 是力矩，单位是 N·m。模型只有连杆质量和惯量，没有电机转子惯量、减速比、摩擦，也没有控制器内部保存的绝对加速度上限。

## 1. 已经有的力矩

这些数来自 [Gento_Sky_mj.urdf](../apex/install/marvin_description/share/marvin_description/urdf/Gento_Sky/Gento_Sky_mj.urdf) 的 `effort`，并与 [Gento_Sky.xml](../apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml) 的 `actuatorfrcrange` 一致。左右臂相同。

| 关节 | 力矩 \(\tau_{\max}\) | 模型速度 |
|---|---:|---:|
| 手臂 `Joint1`、`Joint2` | 108 N·m | 3.1416 rad/s |
| 手臂 `Joint3`、`Joint4` | 66 N·m | 3.1416 rad/s |
| 手臂 `Joint5`、`Joint6`、`Joint7` | 18 N·m | 3.1416 rad/s |
| 腰部旋转 `waist_joint_2`、`waist_joint_3` | 194 N·m | 3.14 rad/s |
| 头部 `Head_J1_Joint`、`Head_J2_Joint` | 22 N·m | 3.14 rad/s |
| 升降 `waist_joint_1` | URDF 中 `effort="0"` | 0.2 m/s，来自 URDF 的 `velocity`，MJCF 没有速度 |

MoveIt 里头部叫 `head_joint_1`、`head_joint_2`，升降叫 `waist_jiont_1_prismatic`。`jiont` 是模型里的既有拼写。

当前 YAML 把旋转关节的 `max_acceleration` 写成和 `max_velocity` 相同的数，例如手臂是 3.1416 rad/s²。两者单位不同。数值相同只表示理想匀加速时

$$
t=\frac{v_{\max}}{a_{\max}}=1\ \mathrm{s}
$$

这不说明电机、减速器和带负载的手臂能够在 1 秒内加到该速度。

## 2. 加速度需要的式子

每个关节在姿态 \(q\) 下，可用于加速的力矩要先扣掉重力和摩擦。MoveIt 每个关节只接受一个常数，所以要在任务会经过的姿态里取最小值：

$$
a_{i,\max}=\min_q\frac{\tau_{i,\max}-|\tau_{i,\mathrm{gravity}}(q)|-\tau_{i,\mathrm{friction}}}{H_{ii}(q)+I_{i,\mathrm{rotor}}N_i^2}
$$

| 符号 | 含义 | 仓库里有没有 |
|---|---|---|
| \(\tau_{i,\max}\) | 上表中的关节力矩 | 有，单位 N·m |
| \(\tau_{i,\mathrm{gravity}}(q)\) | 该姿态下的重力力矩 | 可由连杆质量计算，随姿态变化 |
| \(\tau_{i,\mathrm{friction}}\) | 摩擦和额外阻力 | 没有 |
| \(H_{ii}(q)\) | 关节空间质量矩阵的对角项，含夹爪和物体 | 连杆部分在 MJCF 里；夹爪和物体要另加 |
| \(I_{i,\mathrm{rotor}}\) | 电机转子惯量 | 没有 |
| \(N_i\) | 减速比 | 没有 |
| 控制器内部加速度上限 | `accel_ratio: 100` 所对应的分母 | 没有。100 是百分比 |

分母里的 \(I_{i,\mathrm{rotor}}N_i^2\) 往往比单个连杆惯量更大。缺了它，用连杆惯量去除力矩会把加速度算得过大。

升降轴的 `effort` 是 0，上式无法给它加速度。腰部升降在遥操作速度文件里的 `1.0` 是速度帽，不是这条公式的输入。

## 3. 为什么不能拿一个惯量直接除

`Joint7` 连杆的最大主惯量约为 \(0.00174\ \mathrm{kg\cdot m^2}\)。如果只用它去除 18 N·m：

$$
\frac{18}{0.00174}\approx 1.0\times 10^{4}\ \mathrm{rad/s^2}
$$

这个结果漏掉了电机转子、减速比、近端连杆和重力，不能写入 `max_acceleration`。肩关节也会出现同一类问题：伸直后的连杆惯量只是分母的一部分，重力还会占掉一部分 108 N·m。

因此，108、66、18、194、22 都不能改写成 rad/s² 或 m/s² 后填进 YAML。

## 4. 控制器也没有给出这个数

[robot_param_max.yaml](../apex/install/marvin_ros_control/share/marvin_ros_control/config/robot_param_max.yaml) 中 Gento Skye 的配置是 `speed_ratio: 100`、`accel_ratio: 100`。[MarvinSDK.h](../apex/install/marvin_teleop/include/marvin_teleop/MarvinSDK.h) 把它们定义为百分比：`m_Joint_Vel_Ratio` 与 `m_Joint_Acc_Ratio`。`marvin_robot_node` 通过 `FX_L1_Runtime_SetSpeedRatio` 下发，左右臂接口是 `OnSetJointLmt_A(int velRatio, int AccRatio)` 和 `OnSetJointLmt_B`。

100 表示使用控制器内部已经保存的绝对上限。那个绝对上限的 rad/s² 不在 apex 仓库里，头文件也没有 `OnGetFloatPara` / `OnGetIntPara` 所需的参数名。把 `accel_ratio` 改小，只是按未知分母缩放，不会产生一份已知的关节加速度。

同一配置里的这些量也不是 `max_acceleration`：

- `pd_cmd_cycle_time_ms: 10`：位置指令周期，100 Hz。
- `left_pd` / `right_pd` 的 `k`、`d`：阻抗刚度和阻尼。
- `left_tool` / `right_tool`：10 个数的工具动力学参数，文件没有逐项标出物理含义。
- 遥操作的 `control_frequency_hz: 250`：求逆解的循环频率。
- `go_home_joint_speed: 0.3`：回零使用的关节速度。

真机夹爪是 OmniPicker1，速度和加速度是 0 到 255 的协议字节，默认都是 `0x80`。它不使用手臂力矩，也不能换成 MoveIt 指爪关节上的 1.0 m/s²。

## 5. 以后要补上什么才能计算

按这个顺序补数据之后，才能算出一个候选加速度：

1. 厂家给出每个关节的连续力矩、峰值力矩、减速比和电机转子惯量，并确认上表的 108、66、18、194、22 对应哪一种力矩。
2. 用 `Gento_Sky.xml` 的连杆惯量，加上实际夹爪和物体质量，在回零、预抓取、抓取和手臂伸直这些姿态上计算 \(H_{ii}(q)\) 与重力力矩。
3. 连上控制器，读回 `accel_ratio` 所对应的绝对加速度上限。
4. 取第 2 步里最小的加速度，再取它与控制器绝对上限的较小值，并留出余量。
5. 把来源、负载、姿态、软件版本和日期记在配置旁。通过跟踪误差、振动、电流和温升之后，再把它写成 `max_acceleration`。

在这五步完成之前，手臂、腰部旋转和头部可以继续使用与模型一致的速度做演示。YAML 里现有的加速度保持为未验证占位，不把力矩换算后的任何 rad/s² 写进去。
