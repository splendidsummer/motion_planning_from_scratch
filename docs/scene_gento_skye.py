#!/usr/bin/python3
"""Put a table, a grasp cube, and obstacle balls in front of Gento Skye, then plan.

The robot faces +X. The cube sits on the table on the midline, between the two
arms, so a later grasp can reach it from both sides. The balls sit in the
volume an arm crosses on the way to that cube.

Start the demo first:

    ros2 launch gento_skye_moveit_config demo.launch.py

Then:

    ./scene_gento_skye.py
    ./scene_gento_skye.py --planner RRTstar --time 8
"""

import argparse
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    ObjectColor,
    PlanningScene,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectoryPoint

# 五个主规划器的说明在 tutorials/<名字>.md，其余改法在 tutorials/KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md。
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

# Table top is in front of the torso (torso collision ends near x=0.34).
# waist_Link_3 is at z=0.36, so the top sits at the waist, below the torso column.
# The near edge of the gray rim stays near x=0.44. White top sits on that rim.
TABLE_CENTER_X = 1.25
TABLE_TOP_Z = 0.36
WHITE_SIZE = (1.50, 2.40, 0.05)
GRAY_RIM_SIZE = (1.62, 2.52, 0.03)
LEG_SIZE = 0.09
CUBE_SIZE = 0.06
CUBE_CENTER = (0.70, 0.0, TABLE_TOP_Z + CUBE_SIZE / 2.0)
BALLS = (
    ("ball_right", (0.45, -0.55, 0.86), 0.07),
    ("ball_left", (0.45, 0.55, 0.86), 0.07),
    ("ball_high_right", (0.62, -0.28, 1.02), 0.06),
    ("ball_high_left", (0.62, 0.28, 1.02), 0.06),
    ("ball_front", (0.90, 0.0, 0.98), 0.07),
)

# Cube center is near z=0.39. Pregrasp is outside the cube, grasp is closer,
# retreat lifts the closed hands back toward the robot.
PREGRASP = {
    "right_arm": (
        (0.70, -0.18, 0.52),
        (0.62, -0.22, 0.55),
        (0.55, -0.24, 0.60),
    ),
    "left_arm": (
        (0.70, 0.18, 0.52),
        (0.62, 0.22, 0.55),
        (0.55, 0.24, 0.60),
    ),
}
CURRENT_ORIENTATION = {
    "right_arm": (0.70710678, 0.0, 0.0, 0.70710678),
    "left_arm": (-0.70710678, 0.0, 0.0, 0.70710678),
}
ARM_JOINTS = {
    "right_arm": [f"Joint{i}_R" for i in range(1, 8)],
    "left_arm": [f"Joint{i}_L" for i in range(1, 8)],
}
TIP = {"right_arm": "TCP_Link_R", "left_arm": "TCP_Link_L"}
# Prismatic finger joints: 0.0 m is wide open (jaws 0.11 m apart),
# 0.012 m holds the 0.06 m cube, and 0.04 m is fully closed.
# The mimic joints are listed too, so the displayed fingers both move.
GRIPPER_JOINTS = ("Arm_R10_Joint", "Arm_R11_Joint", "Arm_L10_Joint", "Arm_L11_Joint")
ARM_GRIPPERS = {
    "right_arm": ("Arm_R10_Joint", "Arm_R11_Joint"),
    "left_arm": ("Arm_L10_Joint", "Arm_L11_Joint"),
}
PINCH_IN_TCP = (0.0, 0.0, 0.169)
FINGER_SPREAD = 0.0
FINGER_HOLD = 0.012
GRIPPER_OPEN = FINGER_SPREAD
GRIPPER_CLOSED = FINGER_HOLD
GRIPPER_FULLY_CLOSED = 0.04


def rotation_to_quat(rotation):
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
            0.25 * scale,
        )
    if rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        return (
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[2][1] - rotation[1][2]) / scale,
        )
    if rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        return (
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
        )
    scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
    return (
        (rotation[0][2] + rotation[2][0]) / scale,
        (rotation[1][2] + rotation[2][1]) / scale,
        0.25 * scale,
        (rotation[1][0] - rotation[0][1]) / scale,
    )


def rotate_vector(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    return (
        vx + 2.0 * (qw * cx + qy * cz - qz * cy),
        vy + 2.0 * (qw * cy + qz * cx - qx * cz),
        vz + 2.0 * (qw * cz + qx * cy - qy * cx),
    )


def quat_from_rpy(roll, pitch, yaw):
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose(xyz, quaternion):
    message = Pose()
    message.position.x, message.position.y, message.position.z = xyz
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = quaternion
    return message


def box(object_id, center, size, color):
    item = CollisionObject()
    item.header.frame_id = "base_link"
    item.id = object_id
    item.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)
    item.primitives.append(primitive)
    item.primitive_poses.append(pose(center, (0.0, 0.0, 0.0, 1.0)))
    paint = ObjectColor()
    paint.id = object_id
    paint.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
    return item, paint


def sphere(object_id, center, radius, color):
    item = CollisionObject()
    item.header.frame_id = "base_link"
    item.id = object_id
    item.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [radius]
    item.primitives.append(primitive)
    item.primitive_poses.append(pose(center, (0.0, 0.0, 0.0, 1.0)))
    paint = ObjectColor()
    paint.id = object_id
    paint.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
    return item, paint


def colored_box(object_id, parts, color):
    item = CollisionObject()
    item.header.frame_id = "base_link"
    item.id = object_id
    item.operation = CollisionObject.ADD
    for center, size in parts:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)
        item.primitives.append(primitive)
        item.primitive_poses.append(pose(center, (0.0, 0.0, 0.0, 1.0)))
    paint = ObjectColor()
    paint.id = object_id
    paint.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
    return item, paint


def table_objects():
    """White top, gray rim, and four gray legs. One color per collision object."""
    white_center_z = TABLE_TOP_Z - WHITE_SIZE[2] / 2.0
    white_bottom = white_center_z - WHITE_SIZE[2] / 2.0
    rim_center_z = white_bottom - GRAY_RIM_SIZE[2] / 2.0
    rim_bottom = rim_center_z - GRAY_RIM_SIZE[2] / 2.0
    top, top_color = colored_box(
        "table_top",
        [((TABLE_CENTER_X, 0.0, white_center_z), WHITE_SIZE)],
        (1.0, 1.0, 1.0),
    )
    legs = []
    inset_x = GRAY_RIM_SIZE[0] / 2.0 - 0.10
    inset_y = GRAY_RIM_SIZE[1] / 2.0 - 0.10
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            legs.append(
                (
                    (TABLE_CENTER_X + sign_x * inset_x, sign_y * inset_y, rim_bottom / 2.0),
                    (LEG_SIZE, LEG_SIZE, rim_bottom),
                )
            )
    frame, frame_color = colored_box(
        "table_legs",
        [((TABLE_CENTER_X, 0.0, rim_center_z), GRAY_RIM_SIZE)] + legs,
        (0.42, 0.42, 0.44),
    )
    return [(top, top_color), (frame, frame_color)]


def scene_objects():
    objects = []
    colors = []
    for table, table_color in table_objects():
        objects.append(table)
        colors.append(table_color)
    cube, cube_color = box("cube", CUBE_CENTER, (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), (0.95, 0.45, 0.05))
    objects.append(cube)
    colors.append(cube_color)
    ball_colors = (
        (0.85, 0.15, 0.15),
        (0.15, 0.35, 0.90),
        (0.95, 0.85, 0.15),
        (0.15, 0.75, 0.85),
        (0.65, 0.20, 0.75),
    )
    for (name, center, radius), color in zip(BALLS, ball_colors):
        ball, ball_color = sphere(name, center, radius, color)
        objects.append(ball)
        colors.append(ball_color)
    return objects, colors


def remove_objects(object_ids):
    removed = []
    for object_id in object_ids:
        item = CollisionObject()
        item.header.frame_id = "base_link"
        item.id = object_id
        item.operation = CollisionObject.REMOVE
        removed.append(item)
    return removed


def call(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise SystemExit(f"{client.srv_name} timed out")
    return future.result()


def apply_scene(node, client):
    names = ["table", "table_top", "table_legs", "cube"] + [name for name, _, _ in BALLS]
    clearing = PlanningScene()
    clearing.is_diff = True
    clearing.world.collision_objects = remove_objects(names)
    # Missing ids are not an error: the first run has nothing to remove.
    call(node, client, ApplyPlanningScene.Request(scene=clearing), 10.0)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects, scene.object_colors = scene_objects()
    applied = call(node, client, ApplyPlanningScene.Request(scene=scene), 10.0)
    if not applied.success:
        raise SystemExit("failed to add the table, cube, and balls")
    print("scene=table,cube,balls")


def open_grippers(joints=None):
    state = {name: GRIPPER_OPEN for name in GRIPPER_JOINTS}
    if joints:
        state.update(joints)
    return state


def write_joints(joint_state, joints):
    names = list(joint_state.name)
    positions = list(joint_state.position)
    for name, position in joints.items():
        if name in names:
            positions[names.index(name)] = float(position)
        else:
            names.append(name)
            positions.append(float(position))
    joint_state.name = names
    joint_state.position = positions


def start_is_valid(node, client, joints=None):
    request = GetStateValidity.Request()
    request.robot_state = RobotState()
    request.robot_state.is_diff = True
    if joints:
        write_joints(request.robot_state.joint_state, joints)
    request.group_name = "both_arms"
    response = call(node, client, request, 10.0)
    print(f"start_valid={response.valid}")
    if not response.valid:
        touched = sorted({contact.contact_body_1 + "/" + contact.contact_body_2 for contact in response.contacts})
        print(f"start_contacts={touched}")
    return response.valid


def orientations_for(group):
    current = CURRENT_ORIENTATION[group]
    found = [current]
    for roll in (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi):
        for pitch in (0.0, math.pi / 2.0, -math.pi / 2.0):
            for yaw in (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi):
                found.append(quat_from_rpy(roll, pitch, yaw))
    return found


def link_position(node, client, link, joints):
    request = GetPositionFK.Request()
    request.header.frame_id = "base_link"
    request.fk_link_names = [link]
    request.robot_state = RobotState()
    request.robot_state.joint_state.name = list(joints)
    request.robot_state.joint_state.position = [float(joints[name]) for name in joints]
    response = call(node, client, request, 10.0)
    if response.error_code.val != 1 or not response.pose_stamped:
        return None
    position = response.pose_stamped[0].pose.position
    return (position.x, position.y, position.z)


def elbow_in_front(node, client, group, joints):
    """Reject poses that flip the elbow over the shoulder or behind the chest."""
    link = "Link4_R" if group == "right_arm" else "Link4_L"
    position = link_position(node, client, link, joints)
    if position is None:
        return False
    side = -1.0 if group == "right_arm" else 1.0
    x, y, z = position
    return x > 0.25 and z < 1.05 and y * side > -0.05


def arm_joint_names():
    return ARM_JOINTS["right_arm"] + ARM_JOINTS["left_arm"]


def joint_distance(left, right, names=None):
    names = arm_joint_names() if names is None else names
    return math.sqrt(sum((float(left.get(name, 0.0)) - float(right.get(name, 0.0))) ** 2 for name in names))


def solve_ik(
    node, client, group, fk_client, seed=None, limit=2, xyz_list=None,
    gripper=GRIPPER_OPEN, preferred=None, nearby=False, lock_orientation=False,
):
    """Collision-free joint solutions nearest to seed. seed is held while this arm moves."""
    request = GetPositionIK.Request()
    ik = request.ik_request
    ik.group_name = group
    ik.ik_link_name = TIP[group]
    ik.robot_state = RobotState()
    ik.robot_state.is_diff = True
    held = {name: gripper for name in GRIPPER_JOINTS}
    held.update({name: 0.0 for name in arm_joint_names()})
    if seed:
        held.update(seed)
    write_joints(ik.robot_state.joint_state, held)
    # Collision is checked afterwards. KDL returns no solution when this is on
    # and the only poses it finds in a short timeout graze the table or the cube.
    ik.avoid_collisions = False
    ik.timeout.sec = 0
    ik.timeout.nanosec = 250_000_000 if lock_orientation else 80_000_000
    ik.pose_stamped = PoseStamped()
    ik.pose_stamped.header.frame_id = "base_link"
    reference = {name: held.get(name, 0.0) for name in ARM_JOINTS[group]}
    found = []
    targets = xyz_list if xyz_list is not None else PREGRASP[group]
    if lock_orientation and preferred is not None:
        orientations = [preferred]
    else:
        orientations = []
        if preferred is not None:
            orientations.append(preferred)
        for quaternion in orientations_for(group):
            if preferred is None or quaternion != preferred:
                orientations.append(quaternion)
    accept = 0.55 if nearby else 0.8
    give_up = 24 if nearby else 8
    checked = 0
    for xyz in targets:
        for quaternion in orientations:
            checked += 1
            ik.pose_stamped.pose = pose(xyz, quaternion)
            response = call(node, client, request, 5.0)
            if response.error_code.val != 1:
                if checked >= give_up and found:
                    break
                continue
            joints = {}
            names = response.solution.joint_state.name
            positions = response.solution.joint_state.position
            for name in ARM_JOINTS[group]:
                joints[name] = positions[names.index(name)]
            if not elbow_in_front(node, fk_client, group, {**held, **joints}):
                if lock_orientation:
                    print(f"  {group} elbow_rejected")
                if checked >= give_up and found:
                    break
                continue
            distance = joint_distance(reference, joints, ARM_JOINTS[group])
            print(f"ik={group} xyz={[round(value, 3) for value in xyz]} joint_distance={distance:.3f}")
            found.append((distance, joints, quaternion))
            found.sort(key=lambda item: item[0])
            if found[0][0] < accept or (checked >= give_up and found):
                return [(item[1], item[2]) for item in found[:limit]]
        if found:
            break
    found.sort(key=lambda item: item[0])
    return [(item[1], item[2]) for item in found[:limit]]


def goal_is_valid(node, client, joints, gripper=GRIPPER_CLOSED, quiet=False):
    request = GetStateValidity.Request()
    request.robot_state = RobotState()
    request.robot_state.is_diff = True
    state = {name: gripper for name in GRIPPER_JOINTS}
    state.update(joints)
    write_joints(request.robot_state.joint_state, state)
    request.group_name = "both_arms"
    response = call(node, client, request, 10.0)
    if not quiet:
        print(f"goal_valid={response.valid}")
    if not response.valid and not quiet:
        touched = sorted({contact.contact_body_1 + "/" + contact.contact_body_2 for contact in response.contacts})
        print(f"goal_contacts={touched}")
    return response.valid


def fk_tip(node, client, group, joints):
    request = GetPositionFK.Request()
    request.header.frame_id = "base_link"
    request.fk_link_names = [TIP[group]]
    request.robot_state = RobotState()
    request.robot_state.joint_state.name = list(joints)
    request.robot_state.joint_state.position = [joints[name] for name in joints]
    response = call(node, client, request, 10.0)
    if response.error_code.val != 1 or not response.pose_stamped:
        return None
    position = response.pose_stamped[0].pose.position
    return (position.x, position.y, position.z)


def plan_group(node, client, group, joints, planner, allowed_time, start_joints=None, gripper=GRIPPER_CLOSED):
    request = GetMotionPlan.Request()
    plan = request.motion_plan_request
    plan.group_name = group
    plan.planner_id = planner
    plan.pipeline_id = "ompl"
    plan.num_planning_attempts = 8
    plan.allowed_planning_time = allowed_time
    plan.max_velocity_scaling_factor = 0.25
    plan.max_acceleration_scaling_factor = 0.25
    plan.start_state.is_diff = True
    state = {name: 0.0 for name in ARM_JOINTS["right_arm"] + ARM_JOINTS["left_arm"]}
    state.update({name: gripper for name in GRIPPER_JOINTS})
    if start_joints:
        state.update(start_joints)
    write_joints(plan.start_state.joint_state, state)
    goal = Constraints()
    for name, position in joints.items():
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(position)
        joint.tolerance_above = 0.05
        joint.tolerance_below = 0.05
        joint.weight = 1.0
        goal.joint_constraints.append(joint)
    plan.goal_constraints.append(goal)
    response = call(node, client, request, allowed_time + 20.0)
    code = response.motion_plan_response.error_code.val
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    print(f"group={group}")
    print(f"planner={planner}")
    print(f"error_code={code}")
    print(f"points={len(trajectory.points)}")
    if trajectory.points:
        last = trajectory.points[-1]
        duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        print(f"duration_sec={duration:.3f}")
    return response if code == 1 and len(trajectory.points) >= 2 else None


def interpolated_motion(node, validity_client, start, goal, gripper, group="both_arms"):
    """Move each joint in a straight line when that motion stays collision-free.

    The pre-grasp and grasp targets are a few centimeters apart. Sampling-based
    planning turns that into a long arm sweep that hits the obstacle balls.
    """
    names = [name for name in arm_joint_names() if name in goal]
    start = {name: float(start.get(name, 0.0)) for name in names}
    goal = {name: float(goal[name]) for name in names}
    peak = max(abs(goal[name] - start[name]) for name in names)
    distance = joint_distance(start, goal, names)
    steps = max(4, int(math.ceil(max(peak, 1e-4) / 0.08)))
    if steps > 50:
        print(f"joint_interpolation_skipped distance={distance:.3f}")
        return None
    samples = []
    for index in range(steps + 1):
        blend = index / steps
        samples.append({name: start[name] + blend * (goal[name] - start[name]) for name in names})
    for sample in samples:
        if not goal_is_valid(node, validity_client, sample, gripper, quiet=True):
            print(f"joint_interpolation_blocked distance={distance:.3f}")
            return None
    duration = max(peak / (0.25 * math.pi), 0.6)
    response = GetMotionPlan.Response()
    response.motion_plan_response.error_code.val = 1
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    trajectory.joint_names = list(names)
    for index, sample in enumerate(samples):
        point = JointTrajectoryPoint()
        point.positions = [sample[name] for name in names]
        if 0 < index < steps:
            point.velocities = [(goal[name] - start[name]) / duration for name in names]
        else:
            point.velocities = [0.0] * len(names)
        point.accelerations = [0.0] * len(names)
        stamp_at(point, duration * index / steps)
        trajectory.points.append(point)
    print(f"group={group}")
    print("planner=joint_interpolation")
    print("error_code=1")
    print(f"points={len(samples)}")
    print(f"duration_sec={duration:.3f}")
    print(f"joint_distance={distance:.3f}")
    return response


def endpoint_joints(response):
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    last = trajectory.points[-1]
    return {
        name: position
        for name, position in zip(trajectory.joint_names, last.positions)
        if name in ARM_JOINTS["right_arm"] or name in ARM_JOINTS["left_arm"]
    }


def plan_between(
    node, plan_client, validity_client, goal, planner, allowed_time, start, gripper, group,
    allow_interpolation=False,
):
    if allow_interpolation:
        direct = interpolated_motion(node, validity_client, start, goal, gripper, group)
        if direct is not None:
            return direct
    return plan_group(
        node, plan_client, group, goal, planner, allowed_time,
        start_joints=start, gripper=gripper,
    )


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_at(point, when):
    point.time_from_start.sec = int(when)
    point.time_from_start.nanosec = int(round((when - int(when)) * 1e9))


def append_gripper_close(response):
    """Keep the fingers fully open during the arm motion, then close them."""
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    names = list(trajectory.joint_names)
    for name in GRIPPER_JOINTS:
        if name not in names:
            names.append(name)
    gripper_index = {name: names.index(name) for name in GRIPPER_JOINTS}
    rebuilt = []
    for point in trajectory.points:
        values = [0.0] * len(names)
        velocities = [0.0] * len(names)
        accelerations = [0.0] * len(names)
        for index, name in enumerate(trajectory.joint_names):
            target_idx = names.index(name)
            values[target_idx] = point.positions[index]
            if index < len(point.velocities):
                velocities[target_idx] = point.velocities[index]
            if index < len(point.accelerations):
                accelerations[target_idx] = point.accelerations[index]
        for index in gripper_index.values():
            values[index] = GRIPPER_OPEN
            velocities[index] = 0.0
            accelerations[index] = 0.0
        rebuilt_point = JointTrajectoryPoint()
        rebuilt_point.positions = values
        rebuilt_point.velocities = velocities
        rebuilt_point.accelerations = accelerations
        rebuilt_point.time_from_start = point.time_from_start
        rebuilt.append(rebuilt_point)
    start_time = seconds(rebuilt[-1].time_from_start)
    close_duration = 1.0
    steps = 8
    close_rate = (GRIPPER_CLOSED - GRIPPER_OPEN) / close_duration
    for step in range(1, steps + 1):
        closing = GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * step / steps
        rebuilt_point = JointTrajectoryPoint()
        rebuilt_point.positions = list(rebuilt[-1].positions)
        rebuilt_point.velocities = [0.0] * len(names)
        rebuilt_point.accelerations = [0.0] * len(names)
        for index in gripper_index.values():
            rebuilt_point.positions[index] = closing
            rebuilt_point.velocities[index] = 0.0 if step == steps else close_rate
        stamp_at(rebuilt_point, start_time + close_duration * step / steps)
        rebuilt.append(rebuilt_point)
    trajectory.joint_names = names
    trajectory.points = rebuilt
    write_joints(response.motion_plan_response.trajectory_start.joint_state, open_grippers())
    print(f"grippers={GRIPPER_OPEN:.2f}_to_{GRIPPER_CLOSED:.2f}")


def current_joint_state(node):
    holder = {}

    def remember(message):
        holder["message"] = message

    subscription = node.create_subscription(JointState, "/joint_states", remember, qos_profile_sensor_data)
    deadline = time.time() + 2.0
    while time.time() < deadline and "message" not in holder:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    if "message" not in holder:
        raise SystemExit("no /joint_states received")
    return holder["message"]


def joint_state_publisher_pids():
    output = subprocess.run(
        ["pgrep", "-f", "/joint_state_publisher/joint_state_publisher"],
        check=False,
        capture_output=True,
        text=True,
    )
    return [int(line) for line in output.stdout.split()]


def signal_joint_state_publisher(sig):
    touched = []
    for pid in joint_state_publisher_pids():
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            continue
        touched.append(pid)
    return touched


def pause_joint_state_publisher():
    """Stop the default publisher so it does not pull the arms back to zero."""
    return signal_joint_state_publisher(signal.SIGSTOP)


def publish_positions(node, publisher, positions):
    message = JointState()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = list(positions)
    message.position = [positions[name] for name in message.name]
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.0)


def reset_to_home(node):
    """Put both arms back out to the sides before planning again."""
    signal_joint_state_publisher(signal.SIGCONT)
    time.sleep(0.3)
    baseline = current_joint_state(node)
    positions = dict(zip(baseline.name, baseline.position))
    pause_joint_state_publFINGER_SPREAD
    publisher = node.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
    deadline = time.time() + 0.4
    while time.time() < deadline:
        publish_positions(node, publisher, positions)
        time.sleep(0.05)
    print("reset=arms_out grippers=sprea
    while time.time() < deadline:
        publish_positions(node, publisher, positions)
        time.sleep(0.05)
    print("reset=arms_out grippers=closed")


def link_pose(node, client, link, joints):
    request = GetPositionFK.Request()
    request.header.frame_id = "base_link"
    request.fk_link_names = [link]
    request.robot_state = RobotState()
    request.robot_state.joint_state.name = list(joints)
    request.robot_state.joint_state.position = [float(joints[name]) for name in joints]
    response = call(node, client, request, 10.0)
    if response.error_code.val != 1 or not response.pose_stamped:
        return None, None
    pose_msg = response.pose_stamped[0].pose
    position = (pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)
    quaternion = (pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w)
    return position, quaternion


def play_on_model(node, response, scene_client=None, carry_after=None, fk_client=None, carry_arm=None):
    """Publish the planned joints so the solid robot in RViz follows them."""
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    signal_joint_state_publisher(signal.SIGCONT)
    baseline = current_joint_state(node)
    positions = dict(zip(baseline.name, baseline.position))
    paused = pause_joint_state_publisher()
    publisher = node.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
    started = time.monotonic()
    carried = None
    for point in trajectory.points:
        for name, position in zip(trajectory.joint_names, point.positions):
            positions[name] = position
        wait = seconds(point.time_from_start) - (time.monotonic() - started)
        if wait > 0:
            time.sleep(wait)
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = list(positions)
        message.position = [positions[name] for name in message.name]
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)
        if scene_client is not None and carry_arm is not None and seconds(point.time_from_start) >= carry_after:
            joints = dict(zip(trajectory.joint_names, point.positions))
            tcp_position, tcp_quaternion = link_pose(node, fk_client, TIP[carry_arm], joints)
            if tcp_position is not None:
                pinch = tuple(
                    tcp_position[index] + offset
                    for index, offset in enumerate(rotate_vector(tcp_quaternion, PINCH_IN_TCP))
                )
                if carried is None:
                    carried = tuple(CUBE_CENTER[index] - pinch[index] for index in range(3))
                move_cube(node, scene_client, tuple(pinch[index] + carried[index] for index in range(3)))
    print(f"played_on_model=true paused_joint_state_publisher={paused}")


FULL_NAMES = ARM_JOINTS["left_arm"] + ARM_JOINTS["right_arm"] + list(GRIPPER_JOINTS)


def point_at(
    source_names, source_values, gripper, when, gripper_joints=None,
    source_velocities=None, source_accelerations=None,
):
    values = [0.0] * len(FULL_NAMES)
    velocities = [0.0] * len(FULL_NAMES)
    accelerations = [0.0] * len(FULL_NAMES)
    index = {name: i for i, name in enumerate(FULL_NAMES)}
    for name, value in zip(source_names, source_values):
        if name in index:
            values[index[name]] = value
    if source_velocities:
        for name, value in zip(source_names, source_velocities):
            if name in index:
                velocities[index[name]] = value
    if source_accelerations:
        for name, value in zip(source_names, source_accelerations):
            if name in index:
                accelerations[index[name]] = value
    for name in (GRIPPER_JOINTS if gripper_joints is None else gripper_joints):
        values[index[name]] = gripper
        velocities[index[name]] = 0.0
        accelerations[index[name]] = 0.0
    point = JointTrajectoryPoint()
    point.positions = values
    point.velocities = velocities
    point.accelerations = accelerations
    stamp_at(point, when)
    return point


def take_segment(trajectory, gripper, time_offset, gripper_joints=None):
    return [
        point_at(
            trajectory.joint_names,
            point.positions,
            gripper,
            time_offset + seconds(point.time_from_start),
            gripper_joints,
            point.velocities,
            point.accelerations,
        )
        for point in trajectory.points
    ]


def ramp_grippers(last, start, end, time_offset, joints, duration=0.8, steps=6):
    points = []
    rate = (end - start) / duration if duration > 0.0 else 0.0
    for step in range(1, steps + 1):
        opening = start + (end - start) * step / steps
        point = JointTrajectoryPoint()
        point.positions = list(last.positions)
        point.velocities = [0.0] * len(FULL_NAMES)
        point.accelerations = [0.0] * len(FULL_NAMES)
        for name in joints:
            idx = FULL_NAMES.index(name)
            point.positions[idx] = opening
            point.velocities[idx] = 0.0 if step == steps else rate
        stamp_at(point, time_offset + duration * step / steps)
        points.append(point)
    return points


def install_trajectory(response, points):
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    trajectory.joint_names = list(FULL_NAMES)
    trajectory.points = points
    home = {name: 0.0 for name in ARM_JOINTS["right_arm"] + ARM_JOINTS["left_arm"]}
    home.update({name: FINGER_SPREAD for name in GRIPPER_JOINTS})
    write_joints(response.motion_plan_response.trajectory_start.joint_state, home)


def move_cube(node, client, xyz):
    item = CollisionObject()
    item.header.frame_id = "base_link"
    item.id = "cube"
    item.operation = CollisionObject.MOVE
    item.pose = pose(xyz, (0.0, 0.0, 0.0, 1.0))
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(item)
    call(node, client, ApplyPlanningScene.Request(scene=scene), 2.0)


def solve_pair(
    node, ik_client, fk_client, validity_client, right_targets, left_targets, gripper,
    seed=None, right_quat=None, left_quat=None, nearby=False,
):
    """Pick the valid pair closest to seed so the next motion does not reconfigure the arms."""
    seed = {} if seed is None else dict(seed)
    best = None
    best_distance = math.inf
    accept = 0.9 if nearby else 1.2
    for right, right_orientation in solve_ik(
        node, ik_client, "right_arm", fk_client, seed=seed, xyz_list=right_targets,
        gripper=gripper, preferred=right_quat, nearby=nearby,
    ):
        held = dict(seed)
        held.update(right)
        for left, left_orientation in solve_ik(
            node, ik_client, "left_arm", fk_client, seed=held, xyz_list=left_targets,
            gripper=gripper, preferred=left_quat, nearby=nearby,
        ):
            both = {}
            both.update(right)
            both.update(left)
            distance = joint_distance(seed, both)
            if distance >= best_distance:
                continue
            if not goal_is_valid(node, validity_client, both, gripper):
                continue
            best = (both, right_orientation, left_orientation)
            best_distance = distance
            print(f"pair_joint_distance={distance:.3f}")
            if distance < accept:
                return best
    return best


def require_rviz(node):
    """Stop before planning when nothing is listening for the trajectory."""
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
    print("检查 RViz: 等待 /display_planned_path 的订阅者 ...", flush=True)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() >= 1:
            print(f"rviz_listening subscribers={publisher.get_subscription_count()}")
            return publisher
    raise SystemExit(
        "错误: RViz 没有在运行，没有人订阅 /display_planned_path。"
        "请先打开 RViz 窗口，再重新运行。"
    )


def show(node, response):
    shown = DisplayTrajectory()
    shown.model_id = "Gento Skye urdf"
    shown.trajectory_start = response.motion_plan_response.trajectory_start
    shown.trajectory.append(response.motion_plan_response.trajectory)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() >= 1:
            break
    for _ in range(8):
        publisher.publish(shown)
        rclpy.spin_once(node, timeout_sec=0.15)
    subscribers = publisher.get_subscription_count()
    print(f"displayed=/display_planned_path subscribers={subscribers}")
    if subscribers < 1:
        raise SystemExit("RViz 没有在听 /display_planned_path，窗口没有打开。")


def down_quaternion(yaw):
    """Fingers point down. yaw spins the jaws in the horizontal plane."""
    finger = (0.0, 0.0, -1.0)
    opening = (math.cos(yaw), math.sin(yaw), 0.0)
    palm = (
        opening[1] * finger[2] - opening[2] * finger[1],
        opening[2] * finger[0] - opening[0] * finger[2],
        opening[0] * finger[1] - opening[1] * finger[0],
    )
    return rotation_to_quat((
        (palm[0], opening[0], finger[0]),
        (palm[1], opening[1], finger[1]),
        (palm[2], opening[2], finger[2]),
    ))


def tcp_above(pinch):
    """TCP sits above the pinch because the fingers point down."""
    return (pinch[0], pinch[1], pinch[2] + PINCH_IN_TCP[2])


def solve_side(
    node, ik_client, fk_client, validity_client, arm, xyz, quaternion, seed, gripper,
    stage="target",
):
    found = solve_ik(
        node, ik_client, arm, fk_client, seed=seed, xyz_list=(xyz,), gripper=gripper,
        preferred=quaternion, lock_orientation=True, limit=1,
    )
    if not found:
        print(f"  {arm} stage={stage} no_ik xyz={[round(value, 3) for value in xyz]}")
        return None
    for joints, _orientation in found:
        full = {name: 0.0 for name in arm_joint_names()}
        full.update(seed)
        full.update(joints)
        if goal_is_valid(node, validity_client, full, gripper):
            return joints
        print(f"  {arm} stage={stage} goal_collision xyz={[round(value, 3) for value in xyz]}")
    return None


def choose_arm(node, ik_client, fk_client, validity_client, home):
    """One arm reaches down onto the cube. The shorter reach is kept."""
    cx, cy, cz = CUBE_CENTER
    pinches = {
        "pregrasp": (cx, cy, cz + 0.12),
        "grasp": CUBE_CENTER,
        "lift": (cx - 0.12, cy, cz + 0.16),
    }
    best = None
    for arm in ("right_arm", "left_arm"):
        arm_best = None
        rejected_counts = {"pregrasp": 0, "grasp": 0, "grasp_hold": 0, "lift": 0}
        for yaw in (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi / 4.0, -math.pi / 4.0, math.pi):
            quaternion = down_quaternion(yaw)
            pregrasp = solve_side(
                node, ik_client, fk_client, validity_client, arm,
                tcp_above(pinches["pregrasp"]), quaternion, home, FINGER_SPREAD,
                stage="pregrasp",
            )
            if pregrasp is None:
                rejected_counts["pregrasp"] += 1
                continue
            grasp = solve_side(
                node, ik_client, fk_client, validity_client, arm,
                tcp_above(pinches["grasp"]), quaternion, pregrasp, FINGER_SPREAD,
                stage="grasp",
            )
            if grasp is None:
                rejected_counts["grasp"] += 1
                continue
            full_grasp = {name: 0.0 for name in arm_joint_names()}
            full_grasp.update(grasp)
            if not goal_is_valid(node, validity_client, full_grasp, FINGER_HOLD):
                print(f"  {arm} stage=grasp_hold goal_collision yaw_deg={math.degrees(yaw):.0f}")
                rejected_counts["grasp_hold"] += 1
                continue
            lift = solve_side(
                node, ik_client, fk_client, validity_client, arm,
                tcp_above(pinches["lift"]), quaternion, grasp, FINGER_HOLD,
                stage="lift",
            )
            if lift is None:
                rejected_counts["lift"] += 1
                continue
            distance = joint_distance(home, grasp, ARM_JOINTS[arm])
            print(f"candidate={arm} yaw_deg={math.degrees(yaw):.0f} grasp_distance={distance:.3f}")
            if arm_best is None or distance < arm_best["distance"]:
                arm_best = {
                    "arm": arm,
                    "distance": distance,
                    "pregrasp": pregrasp,
                    "grasp": grasp,
                    "lift": lift,
                }
        if arm_best is None:
            print(
                f"candidate={arm} rejected "
                f"pregrasp_fails={rejected_counts['pregrasp']} "
                f"grasp_fails={rejected_counts['grasp']} "
                f"grasp_hold_fails={rejected_counts['grasp_hold']} "
                f"lift_fails={rejected_counts['lift']}"
            )
            continue
        if best is None or arm_best["distance"] < best["distance"]:
            best = arm_best
    return best


def main():
    parser = argparse.ArgumentParser(description="Add a table scene and plan around the balls.")
    parser.add_argument("--planner", choices=PLANNERS, default="RRTConnect")
    parser.add_argument("--time", type=float, default=8.0, help="seconds the planner may run")
    parser.add_argument(
        "--allow-interpolation",
        action="store_true",
        help="allow client-side straight-line joint interpolation before calling OMPL",
    )
    parser.add_argument(
        "--scene-only",
        action="store_true",
        help="add the table scene and return the arms home, without planning",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("scene_gento_skye")
    scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    fk_client = node.create_client(GetPositionFK, "/compute_fk")
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    clients = (
        ("/apply_planning_scene", scene_client),
        ("/check_state_validity", validity_client),
        ("/compute_ik", ik_client),
        ("/compute_fk", fk_client),
        ("/plan_kinematic_path", plan_client),
    )
    for service_name, client in clients:
        if not client.wait_for_service(timeout_sec=30.0):
            raise SystemExit(
                f"{service_name} is not available. Launch demo.launch.py first."
            )

    played = False
    hold_start = False
    try:
        require_rviz(node)
        apply_scene(node, scene_client)
        reset_to_home(node)
        if not start_is_valid(node, validity_client, {name: FINGER_SPREAD for name in GRIPPER_JOINTS}):
            raise SystemExit("the robot already touches the new objects")
        if args.scene_only:
            hold_start = True
            print("ready=scene_and_home")
            print("planning has not started")
            return

        home = {name: 0.0 for name in arm_joint_names()}
        chosen = choose_arm(node, ik_client, fk_client, validity_client, home)
        if chosen is None:
            raise SystemExit("neither arm has a collision-free grasp of the cube")
        arm = chosen["arm"]
        fingers = ARM_GRIPPERS[arm]
        print(f"selected={arm} grasp_distance={chosen['distance']:.3f}")

        print("stage=pregrasp")
        approach = plan_between(
            node, plan_client, validity_client, chosen["pregrasp"], args.planner, args.time,
            home, FINGER_SPREAD, arm, allow_interpolation=args.allow_interpolation,
        )
        if approach is None:
            raise SystemExit("approach to the pre-grasp failed")
        arrived = endpoint_joints(approach)
        arrived_arm = {name: arrived.get(name, chosen["pregrasp"][name]) for name in ARM_JOINTS[arm]}

        print("stage=grasp")
        move_in = plan_between(
            node, plan_client, validity_client, chosen["grasp"], args.planner, args.time,
            arrived_arm, FINGER_SPREAD, arm, allow_interpolation=args.allow_interpolation,
        )
        if move_in is None:
            raise SystemExit("move from the pre-grasp to the grasp failed")
        held = endpoint_joints(move_in)
        held_arm = {name: held.get(name, chosen["grasp"][name]) for name in ARM_JOINTS[arm]}
        full_held = {name: 0.0 for name in arm_joint_names()}
        full_held.update(held_arm)
        if not goal_is_valid(node, validity_client, full_held, FINGER_HOLD):
            raise SystemExit("closing fingers to FINGER_HOLD at arrived grasp state collides")

        print("stage=retreat")
        lift = plan_between(
            node, plan_client, validity_client, chosen["lift"], args.planner, args.time,
            held_arm, FINGER_HOLD, arm, allow_interpolation=args.allow_interpolation,
        )
        if lift is None:
            raise SystemExit("lift after the grasp failed")

        pieces = take_segment(
            approach.motion_plan_response.trajectory.joint_trajectory,
            FINGER_SPREAD, 0.0, fingers,
        )
        moved = take_segment(
            move_in.motion_plan_response.trajectory.joint_trajectory,
            FINGER_SPREAD,
            seconds(pieces[-1].time_from_start),
            fingers,
        )
        if moved and abs(seconds(pieces[-1].time_from_start) - seconds(moved[0].time_from_start)) < 1e-4:
            moved = moved[1:]
        pieces.extend(moved)
        closed = ramp_grippers(
            pieces[-1], FINGER_SPREAD, FINGER_HOLD, seconds(pieces[-1].time_from_start), fingers,
        )
        pieces.extend(closed)
        carry_after = seconds(pieces[-1].time_from_start)
        lifted = take_segment(
            lift.motion_plan_response.trajectory.joint_trajectory,
            FINGER_HOLD,
            carry_after,
            fingers,
        )
        if lifted and abs(seconds(pieces[-1].time_from_start) - seconds(lifted[0].time_from_start)) < 1e-4:
            lifted = lifted[1:]
        pieces.extend(lifted)
        install_trajectory(approach, pieces)
        print(f"grasp_sequence arm={arm} points={len(pieces)} duration_sec={seconds(pieces[-1].time_from_start):.3f}")
        show(node, approach)
        play_on_model(node, approach, scene_client, carry_after, fk_client, arm)
        played = True
    finally:
        if not played and not hold_start:
            signal_joint_state_publisher(signal.SIGCONT)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
