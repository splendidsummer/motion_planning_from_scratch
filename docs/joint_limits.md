# Gento Skye 关节限制说明

本文说明 [MoveIt 关节限制配置](../gento_skye_moveit_config/config/joint_limits.yaml) 中每个字段的含义、当前数值来源，以及带夹爪 Gento Skye 桌面规划应如何使用它们。

## 关键结论

[joint_limits.yaml](../gento_skye_moveit_config/config/joint_limits.yaml) 里的数分成两类。手臂速度 **3.1416 rad/s** 与 Gento Skye 模型和 [apex](../apex) 控制栈一致，可以保留。加速度，以及夹爪的 1.0 m/s、右前轮 `YQL` 的 1.0 rad/s、底盘轮组的 6.28，没有在模型或控制器里找到同样单位的硬件依据。下面按“这份 YAML 管什么、控制器实际收什么、哪些数能对上、哪些数不能照抄”说明。

### 这份 YAML 管的是轨迹时间

桌面规划先在关节空间里搜索一条几何路径，再由 [OMPL 配置](../gento_skye_moveit_config/config/ompl_planning.yaml) 里的 `AddTimeOptimalParameterization` 按 `max_velocity` 和 `max_acceleration` 给这条路径分配时间。[桌面脚本](scene_gento_skye.py) 还把速度和加速度都乘上 `0.25`。手臂配置被加载时，请求里的名义上限约为：

$$
v_{\max,\mathrm{request}}=0.25\times 3.1416\approx 0.7854\ \mathrm{rad/s}\approx 45^\circ/\mathrm{s}
$$

$$
a_{\max,\mathrm{request}}=0.25\times 3.1416\approx 0.7854\ \mathrm{rad/s^2}
$$

这是时间参数化允许达到的上限，不是每个关节都会跑到的速度。它不决定 RRT 一次扩展的 `range`，也不决定沿途碰撞检查隔多远看一个状态。把速度调小，只会让播出来的动作变慢。

### 控制器收的是百分比，不是 rad/s

真机栈在 [apex](../apex)。Gento Skye 的启动文件加载 [robot_param_max.yaml](../apex/install/marvin_ros_control/share/marvin_ros_control/config/robot_param_max.yaml)，其中：

```yaml
robot_type: Gento_Skye
speed_ratio: 100
accel_ratio: 100
pd_cmd_cycle_time_ms: 10
```

`marvin_robot_node` 把这两个比例交给 `FX_L1_Runtime_SetSpeedRatio`。[MarvinSDK.h](../apex/install/marvin_teleop/include/marvin_teleop/MarvinSDK.h) 把它们定义成百分比：`m_Joint_Vel_Ratio` 是关节速度限制百分比，`m_Joint_Acc_Ratio` 是关节加速度限制百分比。左右臂接口是 `OnSetJointLmt_A(int velRatio, int AccRatio)` 和 `OnSetJointLmt_B`。当前值 100 表示使用控制器内部已经保存的绝对上限的 100%。

这个绝对上限的 rad/s 和 rad/s² 没有出现在 apex 仓库里。头文件也没有写出可用 `OnGetFloatPara` / `OnGetIntPara` 读取的参数名。要得到分母，需要连上控制器读回这些参数，或向厂家要关节输出端规格。在读到那个分母之前，不能把 YAML 里的 3.1416 说成“控制器已经证明能跑到 180°/s、并且能在 1 秒内加到这个速度”。

同一份配置里还有几项容易和关节限制混在一起：

- `pd_cmd_cycle_time_ms: 10` 是位置指令周期，100 Hz。它规定命令多久更新一次。
- `left_pd` / `right_pd` 的 `k` 和 `d` 是阻抗刚度和阻尼。
- `left_tool` / `right_tool` 是 10 个数的工具动力学参数，文件没有逐项标出质量、质心或惯量。
- 遥操作求解器 [robot_param.yaml](../apex/install/marvin_qp_controller/share/marvin_qp_controller/config/max/robot_param.yaml) 的 `control_frequency_hz: 250` 是求逆解的循环频率；`go_home_joint_speed: 0.3` 是回零动作使用的关节速度。

这条控制链实际包含的关节是：左右臂各 `Joint1` 到 `Joint7`，升降 `waist_joint_1`，腰部旋转 `waist_joint_2` 和 `waist_joint_3`，头部 `Head_J1_Joint` 和 `Head_J2_Joint`。

### 能和模型对齐的速度与力矩

[Gento_Sky_mj.urdf](../apex/install/marvin_description/share/marvin_description/urdf/Gento_Sky/Gento_Sky_mj.urdf) 与 [Gento_Sky.xml](../apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml) 给出位置范围、速度和力矩。URDF 的 `effort` 与 MJCF 的 `actuatorfrcrange` 数值相同。遥操作速度帽 [velocity_limits.json](../apex/install/marvin_qp_controller/share/marvin_qp_controller/config/max/velocity_limits.json) 里，手臂、腰部旋转和头部都是 3.14 rad/s，与模型速度一致。

| 机构 | 模型速度 | 力矩幅值 | MoveIt YAML |
|---|---:|---:|---|
| 手臂 `Joint1`、`Joint2` | 3.1416 rad/s | 108 | 速度 3.1416 rad/s，加速度 3.1416 rad/s² |
| 手臂 `Joint3`、`Joint4` | 3.1416 rad/s | 66 | 同上 |
| 手臂 `Joint5`、`Joint6`、`Joint7` | 3.1416 rad/s | 18 | 同上 |
| `waist_joint_2`、`waist_joint_3` | 3.14 rad/s | 194 | 速度与加速度都写成 3.14 |
| `Head_J1_Joint`、`Head_J2_Joint` | 3.14 rad/s | 22 | MoveIt 名为 `head_joint_1`、`head_joint_2`，数值同为 3.14 |
| 升降 `waist_joint_1` | 0.2 m/s | 模型未给出力上限 | MoveIt 名为 `waist_jiont_1_prismatic`，速度 0.2 m/s，加速度 1.0 m/s² |

左右臂位置范围相同：`Joint1` 为 ±3.1067 rad，`Joint2` 为 ±2.0944 rad，`Joint3` 为 ±3.1067 rad，`Joint4` 为 −2.5307 至 1.0472 rad，`Joint5` 为 ±3.1067 rad，`Joint6` 为 ±1.0472 rad，`Joint7` 为 ±1.5708 rad。原始 SolidWorks 模型把 `Joint1_L` 写成范围 `[0, 0]`、速度 0；[生成脚本](../gento_skye_moveit_config/generate_description.py) 按右臂对称值修复了它。

加速度不能从速度抄出来。YAML 让 \(a_{\max}=v_{\max}\) 时，理想匀加速从静止到上限速度需要 \(t=v_{\max}/a_{\max}=1\) 秒。已知最大力矩 \(\tau_{\max}\) 和该姿态惯量 \(I(q)\) 时，可用加速度满足

$$
|a|\le\frac{\tau_{\max}}{I(q)}
$$

\(I(q)\) 随手臂姿态、夹爪和所抓物体变化，所以 108、66、18 给不出一个固定的 rad/s²。MoveIt 使用的绝对速度和加速度还要落在控制器内部上限之内；百分比保持 100 时，那个内部上限仍以控制器读回值或厂家规格为准。

要把这些力矩变成 YAML 里的一个加速度，按下面的式子取工作空间里的最小值：

$$
a_{i,\max}=\min_q\frac{\tau_{i,\max}-|\tau_{i,\mathrm{gravity}}(q)|-\tau_{i,\mathrm{friction}}}{H_{ii}(q)+I_{i,\mathrm{rotor}}N_i^2}
$$

其中 \(\tau_{i,\max}\) 用模型力矩：手臂 Joint1/2 为 108 N·m，Joint3/4 为 66 N·m，Joint5–7 为 18 N·m；腰部旋转为 194 N·m；头部为 22 N·m。\(H_{ii}(q)\) 是该姿态下关节空间质量矩阵的对角项，来自 [Gento_Sky.xml](../apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml) 的连杆质量和惯量，并加上夹爪与物体。\(I_{i,\mathrm{rotor}}N_i^2\) 是减速器把电机转子惯量放大到关节侧的部分；这份模型没有转子惯量和减速比。重力项和摩擦项会占掉一部分力矩，加速只能使用剩下的部分。

MoveIt 每个关节只接受一个常数，所以要对桌面任务会经过的姿态采样，取上式的最小值，再留出余量。采样至少包括回零、预抓取、抓取和手臂伸直。算出的数还要不超过控制器内部加速度上限乘以 `accel_ratio/100`。升降轴的 URDF `effort` 是 0，这条公式无法给它加速度。

只用某一个连杆自己的惯量去除力矩会得到不能使用的结果。以 `Joint7` 为例，连杆最大主惯量约为 \(0.00174\ \mathrm{kg\cdot m^2}\)，\(18/0.00174\approx 1.0\times 10^{4}\ \mathrm{rad/s^2}\)。这漏掉了转子惯量、前级连杆和重力，不能写入 `max_acceleration`。当前 YAML 里与速度相同的 3.1416 rad/s² 仍然只是未验证的占位，表示理想情况下约 1 秒加到速度上限。按现有数据无法算出可写入配置的加速度，单独说明见 [加速度限制](acceleration_limits.md)。

### 三处不能照抄进真机参数

**腰部升降的 0.2 m/s 来自机器人 URDF 模型。** 三份 URDF 的升降关节都写着 `<limit ... velocity="0.2" />`，单位是 m/s，也就是每秒 20 cm。行程都是 0 到 0.86 m。

| 机器人模型 | 关节名 |
|---|---|
| [gento_skye.urdf](../gento_skye_moveit_config/config/gento_skye.urdf) | `waist_jiont_1_prismatic`。这是 MoveIt 实际加载的模型 |
| [原始带夹爪 URDF](../Gento%20Skye%20urdf%28gripper%29/Gento%20Skye%20urdf/urdf/Gento%20Skye%20urdf.urdf) | `waist_jiont_1_prismatic` |
| [Gento_Sky.urdf](../apex/install/marvin_description/share/marvin_description/urdf/Gento_Sky/Gento_Sky.urdf) 与 [Gento_Sky_mj.urdf](../apex/install/marvin_description/share/marvin_description/urdf/Gento_Sky/Gento_Sky_mj.urdf) | `waist_joint_1` |

[Gento_Sky.xml](../apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml) 只写了行程 `range="0 0.86"`，没有速度。因此 0.2 m/s 的出处是 URDF 的 `velocity`，不是 MJCF。

[joint_limits.yaml](../gento_skye_moveit_config/config/joint_limits.yaml) 的 `max_velocity: 0.2` 是从 MoveIt 这份 URDF 抄来的。同一条 YAML 里的 `max_acceleration: 1.0` 是加速度，单位 m/s²；URDF 没有加速度字段。按 1.0 m/s²，从静止加到 0.2 m/s 需要 0.2 秒。

遥操作文件 [velocity_limits.json](../apex/install/marvin_qp_controller/share/marvin_qp_controller/config/max/velocity_limits.json) 把 `waist_joint_1` 写成 `1.0`。那个文件整份都是速度，所以这是 1.0 m/s，是 URDF 速度的 5 倍，给求逆解用。MoveIt 桌面规划用的是 URDF 的 0.2 m/s 和 YAML 的 1.0 m/s²。控制器的 `speed_ratio: 100` 只是百分比，没有写明真机执行 0.2 m/s 还是 1.0 m/s。当前桌面双臂规划不搜索这个关节。

**真机夹爪不是 YAML 里的 1.0 m/s。** apex 末端是 OmniPicker1，经 SocketCAN 发送 0 到 255 的协议字节。速度、加速度、减速度默认都是 `0x80`，也就是 128。开合位置是 `0x00` 全闭、`0xFF` 全开。它不走 `Joint1` 到 `Joint7` 的关节指令。MoveIt 里的 `Arm_R10_Joint`、`Arm_R11_Joint`、`Arm_L10_Joint`、`Arm_L11_Joint` 是 0 到 0.04 m 的指爪模型，URDF 速度为 0，YAML 另写成 1.0 m/s。按这个速度匀速走完全程只要 0.04 秒。桌面脚本用这四个关节做开合动画；真机开合走的是 OmniPicker 那条 CAN 命令。

**底盘八个轮组关节不在手臂控制器里。** MoveIt 里是左前 `ZQ`/`ZQL`、右前 `YQ`/`YQL`、左后 `ZH`/`ZHL`、右后 `YH`/`YHL`。`marvin_robot_node` 的关节表没有它们。apex 模型 `Gento_Sky_mj.urdf` 里对应车轮关节是 `fixed`，`effort="0"`。因此 YAML 里的 6.28 rad/s 不是这条控制器给出的运行上限。`YQL` 的 URDF 位置范围还是 `[0, 0]`，写成 1.0 rad/s 也不能让它转动。当前桌面任务把 `base_link` 固定在世界上，`both_arms` 不搜索这些关节。

名字要按机构对应：MoveIt 的 `head_joint_1`、`head_joint_2`、`waist_jiont_1_prismatic` 分别对应 apex 的 `Head_J1_Joint`、`Head_J2_Joint`、`waist_joint_1`。`waist_jiont_1_prismatic` 里的 `jiont` 是模型里的既有拼写，改名会使 URDF、SRDF 和 YAML 对不上。

### 现在可以怎样用这些数

演示和时间参数化可以继续使用手臂、腰部旋转和头部与模型一致的速度。加速度保持为待验证配置：在读到控制器内部绝对上限或厂家允许加速度之前，不要把 3.1416 rad/s²、腰部 1.0 m/s²、夹爪 1.0 m/s 或轮组 6.28 rad/s² 写成已验证的真机能力。真机试验时，控制器百分比、MoveIt 绝对上限和末端负载要一起记录；只把 `speed_ratio` 从 100 改小，不会自动改正 YAML 里缺少依据的加速度。

## 1. 哪个文件实际被 MoveIt 使用？

| 文件 | 用途 |
|---|---|
| [joint_limits.yaml](../gento_skye_moveit_config/config/joint_limits.yaml) | MoveIt 的速度、加速度覆盖配置；本文主要说明的文件 |
| [gento_skye.urdf](../gento_skye_moveit_config/config/gento_skye.urdf) | 当前 MoveIt 使用的处理后机器人模型；包含关节类型、位置范围、URDF 速度上限和碰撞几何 |
| [原始带夹爪 URDF](../Gento%20Skye%20urdf%28gripper%29/Gento%20Skye%20urdf/urdf/Gento%20Skye%20urdf.urdf) | SolidWorks 导出的原始模型 |
| [generate_description.py](../gento_skye_moveit_config/generate_description.py) | 从原始 URDF 生成 MoveIt 使用的 URDF 与 SRDF；修复 `Joint1_L` 的错误位置范围 |
| [ompl_planning.yaml](../gento_skye_moveit_config/config/ompl_planning.yaml) | OMPL 插件与规划请求适配器配置，不保存关节速度或加速度限制 |

## 2. YAML 字段与单位

对旋转关节：

```yaml
Joint1_R:
  has_velocity_limits: true
  max_velocity: 3.1416          # rad/s
  has_acceleration_limits: true
  max_acceleration: 3.1416      # rad/s²
```

对直线关节：

```yaml
waist_jiont_1_prismatic:
  has_velocity_limits: true
  max_velocity: 0.2             # m/s
  has_acceleration_limits: true
  max_acceleration: 1.0         # m/s²
```

- `has_velocity_limits: true`：启用 `max_velocity`。
- `max_velocity`：关节速度上限；旋转关节单位为 rad/s，直线关节单位为 m/s。
- `has_acceleration_limits: true`：启用 `max_acceleration`。
- `max_acceleration`：关节加速度上限；旋转关节单位为 rad/s²，直线关节单位为 m/s²。
- 位置上下限不在本 YAML 中定义；通常由 URDF 的 `<limit lower="..." upper="...">` 提供。
- `effort` 也来自 URDF，本 YAML 未覆盖它。

`joint_limits.yaml` 可把 URDF 的限制收紧，不能可靠地放宽 URDF 的零位置范围或其他硬件约束。例如 `YQL` 在 YAML 中写 `1.0` rad/s，但其 URDF 位置范围仍是 `[0, 0]`，所以它不能实际转动。

## 3. 这些限制在哪个阶段起作用？

桌面脚本向 MoveIt 发送几何规划请求，之后由 [OMPL 配置](../gento_skye_moveit_config/config/ompl_planning.yaml) 中的 `AddTimeOptimalParameterization` 为路径分配时间。它使用关节速度和加速度限制，目标是为路径找到满足上限的时间律。

[桌面脚本](scene_gento_skye.py) 为每个规划请求设置：

```python
plan.max_velocity_scaling_factor = 0.25
plan.max_acceleration_scaling_factor = 0.25
```

因此，若手臂的配置限制被加载，手臂请求的名义上限约为：

$$
v_{\max,\mathrm{request}}=0.25\times3.1416\approx0.7854\ \mathrm{rad/s}\approx45^\circ/\mathrm{s}
$$

$$
a_{\max,\mathrm{request}}=0.25\times3.1416\approx0.7854\ \mathrm{rad/s^2}
$$

这两个值是上限，不表示每个关节都会达到该速度或加速度。

### 不由这些数值决定的内容

| 内容 | 实际控制因素 |
|---|---|
| RRT 的一次扩展距离 `range` | OMPL 的规划器参数与状态空间；不是 `max_velocity` |
| 沿途碰撞检查的离散间隔 | `longest_valid_segment_fraction`、`maximum_waypoint_distance` 等状态有效性检查设置；本项目未显式写入这些值 |
| 几何路径是否碰撞 | 机器人碰撞几何、PlanningScene、Allowed Collision Matrix 与沿途状态检查 |
| 真实机器人能否安全执行 | 控制器、驱动器、硬件规格、负载、姿态、线缆、制动距离和实测验证 |

调低速度会让时间参数化后的运动更慢；**不会缩短 RRT 的关节空间边，也不会自动提高碰撞检查分辨率。**

## 4. 当前关节分组

### 4.1 底盘四角轮组

这八个关节位于底盘四个角。根据 [URDF](../gento_skye_moveit_config/config/gento_skye.urdf) 的父子连杆、安装位置和转轴，可将每组理解为一个转向关节和一个车轮滚动关节。以下“左/右、前/后”按 `base_link` 的 $+x$ 向前、$+y$ 向左推断。

| 轮组位置 | 转向关节 | 车轮关节 | 当前 YAML 速度 / 加速度 |
|---|---|---|---|
| 左前 | `ZQ` | `ZQL` | 6.28 rad/s、6.28 rad/s² |
| 右前 | `YQ` | `YQL` | 1.0 rad/s、1.0 rad/s² |
| 左后 | `ZH` | `ZHL` | 6.28 rad/s、6.28 rad/s² |
| 右后 | `YH` | `YHL` | 6.28 rad/s、6.28 rad/s² |

`ZQ`、`YQ`、`ZH`、`YH` 都直接连接到 `base_link`，其局部转轴沿竖直方向，用于改变轮组朝向。末尾带 `L` 的关节连接转向连杆与下方轮连杆；由于安装姿态包含绕 X 轴约 90° 的旋转，局部 Z 轴在基座坐标中对应水平轮轴，表示车轮滚动。

**当前模型中的异常：**

- `YQL` 的 URDF 位置范围为 `[0, 0]` 且 URDF 速度为 `0`；YAML 中的 `1.0` 没有解除这个锁定范围，也没有已知硬件依据。
- 其余车轮关节被表示为有限范围的 `revolute` 关节，约为 ±170°。若真实车轮可以连续滚动，应该由机器人厂商模型确认是否应使用 `continuous` 或其他底盘运动学建模。
- 当前桌面任务的 `both_arms` 不包含这些底盘关节，且 `base_link` 对世界固定；它们不参与双臂桌面 RRT 搜索。

### 4.2 腰部

| 关节 | 类型 | URDF 位置范围 | 当前 YAML 速度 / 加速度 | 说明 |
|---|---|---:|---:|---|
| `waist_jiont_1_prismatic` | 直线 | 0 至 0.86 m | 0.2 m/s、1.0 m/s² | 名字中的 `jiont` 是模型文件既有拼写，不能擅自改名，否则 URDF / SRDF / YAML 不匹配 |
| `waist_joint_2` | 旋转 | −1.5708 至 0 rad | 3.14 rad/s、3.14 rad/s² | 腰部旋转 |
| `waist_joint_3` | 旋转 | −0.4363 至 0.4363 rad | 3.14 rad/s、3.14 rad/s² | 腰部旋转 |

`waist_jiont_1_prismatic` 的 0.2 m/s 来自机器人 URDF 的 `<limit velocity="0.2" />`，MoveIt 模型、原始带夹爪模型和 apex 的 `Gento_Sky.urdf` 都是这个值。YAML 的 `max_velocity: 0.2` 从该字段抄来。1.0 m/s² 是 YAML 另写的加速度，URDF 没有这个字段。

### 4.3 左右手臂

每侧手臂有 7 个旋转关节：`Joint1_R` 至 `Joint7_R`，以及 `Joint1_L` 至 `Joint7_L`。当前 YAML 对全部 14 个手臂关节均设置：

$$
v_{\max}=3.1416\ \mathrm{rad/s}\approx180^\circ/\mathrm{s}
$$

$$
a_{\max}=3.1416\ \mathrm{rad/s^2}
$$

| 关节 | URDF 位置范围（左右相同） | URDF 速度 | YAML 加速度 |
|---|---:|---:|---:|
| `Joint1` | −3.1067 至 3.1067 rad | 3.1416 rad/s | 3.1416 rad/s² |
| `Joint2` | −2.0944 至 2.0944 rad | 3.1416 rad/s | 3.1416 rad/s² |
| `Joint3` | −3.1067 至 3.1067 rad | 3.1416 rad/s | 3.1416 rad/s² |
| `Joint4` | −2.5307 至 1.0472 rad | 3.1416 rad/s | 3.1416 rad/s² |
| `Joint5` | −3.1067 至 3.1067 rad | 3.1416 rad/s | 3.1416 rad/s² |
| `Joint6` | −1.0472 至 1.0472 rad | 3.1416 rad/s | 3.1416 rad/s² |
| `Joint7` | −1.5708 至 1.5708 rad | 3.1416 rad/s | 3.1416 rad/s² |

原始 SolidWorks URDF 错误地将 `Joint1_L` 写为位置范围 `[0, 0]`、速度 `0`。[生成脚本](../gento_skye_moveit_config/generate_description.py) 明确复制 `Joint1_R` 的位置范围、力矩和速度到 `Joint1_L`，基于左右对称修复这个导出错误。其余手臂速度与生成后 URDF 的数值一致。

**加速度限制的来源尚未在仓库中找到。** 将加速度设置为与速度相同的数值，会使理想情况下从静止达到上限速度所需时间为 $t=v_{\max}/a_{\max}=1$ 秒；这只是数学结果，并不验证电机、减速器、结构、负载或控制器能够安全实现该加速度。

### 4.4 夹爪

| 关节 | 类型 | URDF 行程 | 当前 YAML 速度 / 加速度 | 备注 |
|---|---|---:|---:|---|
| `Arm_R10_Joint` | 直线 | 0 至 0.04 m | 1.0 m/s、1.0 m/s² | 右夹爪一侧指爪 |
| `Arm_R11_Joint` | 直线、mimic | 0 至 0.04 m | 1.0 m/s、1.0 m/s² | 右夹爪另一侧指爪；跟随 `Arm_R10_Joint` |
| `Arm_L10_Joint` | 直线 | 0 至 0.04 m | 1.0 m/s、1.0 m/s² | 左夹爪一侧指爪 |
| `Arm_L11_Joint` | 直线、mimic | 0 至 0.04 m | 1.0 m/s、1.0 m/s² | 左夹爪另一侧指爪；跟随 `Arm_L10_Joint` |

URDF 为四个夹爪关节设置了 `velocity="0"`。YAML 中的 1.0 m/s 因此不是从 URDF 复制而来，当前仓库中也没有厂家规格或测试记录作为依据。夹爪行程只有 0.04 m；若按 1.0 m/s 匀速运动，理论上仅需 0.04 秒走完全程，未包含加减速。这显然应在真机使用前以夹爪驱动器规格和实测数据替换。

桌面脚本的手臂路径在各阶段将夹爪固定为 0 或 0.04 m；脚本后来插入的开闭动画没有重新做完整碰撞检查或时间参数化。详见 [RRT 桌面规划说明](../tutorials/RRT.md)。

### 4.5 头部

头部有两个关节，动作不同。`head_joint_1` 绕脖子竖直轴转动，是左右转头。`head_joint_2` 的安装姿态绕 X 轴转了 90°，关节轴变成横向，是点头。真机上低头大约 25°，对上的是点头这一行的一端，不是左右转那一行的 ±90°。

| 关节 | 动作 | URDF 位置范围 | 换成角度 | 当前 YAML 速度 / 加速度 |
|---|---|---:|---:|---:|
| `head_joint_1` | 左右转 | −1.5708 至 1.5708 rad | −90° 至 90° | 3.14 rad/s、3.14 rad/s² |
| `head_joint_2` | 点头 | −0.6109 至 0.4363 rad | −35° 至 25° | 3.14 rad/s、3.14 rad/s² |

`0.4363 rad` 就是 25°。MoveIt URDF、原始带夹爪 URDF 和 apex 的 `Gento_Sky.urdf` 都把点头写成 −35° 到 25°。apex 的 [Gento_Sky.xml](../apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml) 把同一关节写成 −0.2 至 0.4363 rad，也就是大约 −11.5° 到 25°。25° 这一端三份模型一致；另一端 URDF 是 35°，MJCF 大约是 11.5°。模型没有标明正方向是低头还是抬头。真机只确认低头约 25° 时，另一端和左右转的 ±90° 都还要在机器人上核对，不能把表中的整个范围当成已测量的机械限位。

这些角度来自 URDF 的 `lower` / `upper`。YAML 没有改头部位置范围，只写了速度 3.14 rad/s 和未验证的加速度 3.14 rad/s²。头部在单独的 `head` 规划组里，当前桌面 `both_arms` 不搜索它。

## 5. 如何决定可信的真机数值？

按以下优先级确定，而不是从关节角范围、桌子尺寸或 RRT 步长推算：

1. 获取机器人、关节模组、减速器与夹爪的厂家额定速度、连续/峰值扭矩、允许加速度与负载条件。
2. 读取并遵循实际驱动器和底层控制器的硬限位、速度、加速度、加加速度、跟踪误差与制动限制。
3. 用最重末端工具和最大预期负载评估最不利姿态；手臂完全伸展时通常比靠近身体时更受限制。
4. 从显著低于已验证硬限的保守值开始，在安全区域逐级测试位置误差、振动、温度、电流、制动距离和急停行为。
5. 将验证过的限制写入 YAML，保留数据来源、负载、测试姿态、软件版本与日期。

不能由下式单独推出加速度：

$$
a_{\max}=v_{\max}
$$

两者单位不同、物理含义也不同。对于指定速度和加速度，理想的加速时间只是：

$$
t_{\mathrm{accel}}=\frac{v_{\max}}{a_{\max}}
$$

是否允许这个加速过程取决于关节可用力矩与负载惯量，而不取决于这个比值是否“好看”。

### 5.1 对照 apex 之后，每一层分别提供什么

[apex](../apex) 是已安装的 Gento Skye 控制栈，源码目录只有夹爪包；手臂、腰部和头部的模型与控制器配置在 `install/` 中。它把限制分成四层，单位不能互相替代。

| 层 | 文件 | 它实际给出的量 | 能否直接写入 `max_acceleration` |
|---|---|---|---|
| 模型速度与力矩 | [Gento_Sky_mj.urdf](../apex/install/marvin_description/share/marvin_description/urdf/Gento_Sky/Gento_Sky_mj.urdf)、[Gento_Sky.xml](../apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml) | 关节位置范围、速度、`effort` / `actuatorfrcrange` | 不能。力矩要再除以该姿态的惯量才得到加速度 |
| 控制器运行比例 | [robot_param_max.yaml](../apex/install/marvin_ros_control/share/marvin_ros_control/config/robot_param_max.yaml)、[MarvinSDK.h](../apex/install/marvin_teleop/include/marvin_teleop/MarvinSDK.h) | `speed_ratio: 100`、`accel_ratio: 100`，单位是百分比 | 不能。100 表示控制器内部上限的 100%，分母不在这个仓库里 |
| 遥操作速度帽 | [velocity_limits.json](../apex/install/marvin_qp_controller/share/marvin_qp_controller/config/max/velocity_limits.json) | 逆运动学使用的关节速度上限，手臂为 3.14 rad/s | 只约束速度。同文件没有 Skye 的加速度上限 |
| 夹爪协议字节 | [omnipick README](../apex/src/omnipick_gripper_py/README.md) | 速度、加速度、减速度默认都是 `0x80`，范围 `0..255` | 不能换成 1.0 m/s |

Gento Skye 真机启动使用 `robot_param_max.yaml`，`robot_type: Gento_Skye`。`marvin_robot_node` 调用 `FX_L1_Runtime_SetSpeedRatio`，对应 SDK 结构体里的两个字段：

```82:83:apex/install/marvin_teleop/include/marvin_teleop/MarvinSDK.h
	FX_INT16 m_Joint_Vel_Ratio;		///* 关 节 速 度 限 制 百分比*/        2
	FX_INT16 m_Joint_Acc_Ratio;		///* 关 节 加 速 度 限 制  百分比*/    2
```

接口 `OnSetJointLmt_A(int velRatio, int AccRatio)` 和 `OnSetJointLmt_B` 接收的也是这两个整数。当前配置把左右臂、身体和头部都设成 100%。这表示沿用控制器里已经保存的绝对上限，本仓库读不到那个绝对上限的 rad/s 或 rad/s²。要拿到分母，需要在连上控制器后用 SDK 的 `OnGetFloatPara` / `OnGetIntPara` 读取厂家参数名，或向厂家索取关节输出端规格。参数名没有写在这份已安装的头文件里。

同一份控制器配置里还有这些相关量，它们各自约束不同的事情：

- `pd_cmd_cycle_time_ms: 10`：位置指令周期 10 ms，也就是 100 Hz。它限制命令更新有多频繁，不给出允许的关节加速度。
- `left_pd` / `right_pd` 的 `k`、`d`：位置阻抗刚度与阻尼，不是速度或加速度上限。
- `left_tool` / `right_tool`：10 个数的工具动力学参数。YAML 没有给出每个数的物理含义，不能从中反推加速度。
- [QP 参数](../apex/install/marvin_qp_controller/share/marvin_qp_controller/config/max/robot_param.yaml) 的 `control_frequency_hz: 250.0` 是求解循环频率；`go_home_joint_speed: 0.3` 是回零时使用的关节速度，单位与该求解器的关节速度一致，是软件动作速度，不是硬件最大值。

模型里能核对到的手臂输出端数据如下。URDF 的 `effort` 与 MJCF 的 `actuatorfrcrange` 数值相同，按关节力矩使用；速度单位为 rad/s。

| 关节 | 位置范围（rad） | 速度 | 力矩幅值 |
|---|---:|---:|---:|
| `Joint1`、`Joint2`，左右臂相同 | 见 URDF | 3.1416 | 108 |
| `Joint3`、`Joint4` | 见 URDF | 3.1416 | 66 |
| `Joint5`、`Joint6`、`Joint7` | 见 URDF | 3.1416 | 18 |
| `waist_joint_2`、`waist_joint_3` | 见 URDF | 3.14 | 194 |
| `Head_J1_Joint`、`Head_J2_Joint` | 见 URDF | 3.14 | 22 |
| `waist_joint_1`（直线，m） | 0 到 0.86 | URDF `velocity="0.2"` m/s；MJCF 无速度 | URDF 中 `effort="0"`，MJCF 也没有写出力上限 |

因此，MoveIt 手臂 `max_velocity: 3.1416` 可以和这份模型对齐。加速度仍然缺一条独立数据：在已知最大力矩 \(\tau_{\max}\) 和该姿态惯量 \(I(q)\) 时，可用加速度满足

$$
|a| \le \frac{\tau_{\max}}{I(q)}
$$

\(I(q)\) 随姿态、夹爪和所抓物体变化，所以 108、66、18 不能变成一个固定的 `max_acceleration`。在控制器百分比保持 100 的前提下，MoveIt 时间参数化使用的绝对速度、加速度还必须落在控制器内部上限之内；那个内部上限要以控制器读回值或厂家规格为准。

还有三处不能照抄：

- **腰部升降。** 0.2 m/s 写在三份机器人 URDF 的 `velocity` 里，MoveIt 的 `max_velocity` 从那里抄来。遥操作速度文件里的 `1.0` 是另一条 1.0 m/s 的速度帽。YAML 里的另一个 `1.0` 是加速度，单位 m/s²。
- **夹爪。** 真机末端是 OmniPicker1，经 SocketCAN 发送 0 到 255 的速度和加速度字节，默认都是 128。它不走 `Joint1` 到 `Joint7` 的关节指令，也不使用 MoveIt 里 `Arm_*10_Joint` / `Arm_*11_Joint` 的 1.0 m/s。那 1.0 m/s 仍然没有硬件依据。
- **底盘轮组 `ZQ` 到 `YHL`。** `marvin_robot_node` 的关节表只有左右臂 7 关节、`waist_joint_1`、`waist_joint_2/3` 和 `Head_J1/J2`。轮组不在这条控制链里。`Gento_Sky_mj.urdf` 里对应车轮关节还是 `fixed`，`effort="0"`。YAML 里的 6.28 因此不是这条控制器给出的运行上限。

关节名字在两条栈里也不完全相同：MoveIt 使用 `head_joint_1`、`head_joint_2` 和带拼写 `waist_jiont_1_prismatic`；apex 使用 `Head_J1_Joint`、`Head_J2_Joint` 和 `waist_joint_1`。对照数值时按机构对应，不要按字符串完全相等来匹配。

## 6. 修改前的检查清单

- 不要把未知数据填成任意正值以绕过 URDF 的 0；先确认它是导出错误、锁定关节还是硬件限制。
- 修改一个关节名时，同时核查 URDF、SRDF、YAML、控制器映射和脚本；尤其不要把既有的 `waist_jiont_1_prismatic` 拼写直接改成 `joint`。
- 对真实车轮确认连续转动与底盘运动学；不要把双臂 `both_arms` 规划误认为能驱动底盘。
- 对夹爪核实 mimic 关节应否单独下发命令，并核实开合时的碰撞与接触许可。
- 在启动时确认 MoveIt 日志已加载本 YAML；YAML 存在不保证启动文件一定将其传给 `robot_description_planning`。
- 速度和加速度变更后，对最终时间参数化轨迹以及控制器可接受性重新验证。

## 7. 本仓库当前状态总结

- 旋转手臂、腰部旋转、头部和多数轮组的**速度值**与生成后的 URDF 一致，手臂速度也与 apex 中的 `Gento_Sky_mj.urdf` 一致。
- 腰部升降的 0.2 m/s 来自机器人 URDF 的 `velocity="0.2"`，MoveIt、原始带夹爪模型和 apex 的 `Gento_Sky.urdf` 一致。YAML 速度是抄来的；遥操作速度文件中的 `1.0` 是 1.0 m/s 的速度帽。
- `Joint1_L` 的有效范围及速度来自生成脚本的左右对称修复。
- apex 控制器的 `speed_ratio` 和 `accel_ratio` 都是 100%，表示百分比，不能换算成 YAML 里的 rad/s 或 rad/s²。
- 模型给出了手臂、腰部旋转和头部的力矩幅值；加速度仍取决于惯量和负载，仓库中没有对应的 rad/s² 规格。
- `YQL` 和四个夹爪的 YAML 速度值不能由当前 URDF 证明。真机夹爪在 apex 中按 OmniPicker 的 0 到 255 协议字节控制。
- 当前桌面场景是 MoveIt / RViz 演示与模型回放。apex 提供了真机控制入口，但没有把控制器内部的绝对加速度上限带进本仓库。
