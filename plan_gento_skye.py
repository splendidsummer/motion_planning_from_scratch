#!/usr/bin/python3
"""Plan a Gento Skye group with one OMPL algorithm and print the result.

Start the demo first:

    ros2 launch gento_skye_moveit_config demo.launch.py

Then, in another terminal:

    ./plan_gento_skye.py --group both_arms --planner RRTstar
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint
from moveit_msgs.srv import GetMotionPlan

# Names match the planner_configs keys loaded with the OMPL pipeline.
# RViz shows the same names in the MotionPlanning Context tab.
# The five main names are tutorials/<name>.md. The rest are tutorials/KPIECE_BKPIECE_LBKPIECE_SBL_EST_TRRT_BiTRRT_AnytimePathShortening_FITstar.md.
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

# Joints left out stay at the current state. both_arms has no Cartesian IK,
# so a dual-arm goal is a set of joint angles.
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


def build_request(group, planner, allowed_time):
    request = GetMotionPlan.Request()
    plan = request.motion_plan_request
    plan.group_name = group
    plan.planner_id = planner
    plan.pipeline_id = "ompl"
    plan.num_planning_attempts = 4
    plan.allowed_planning_time = allowed_time
    plan.max_velocity_scaling_factor = 0.25
    plan.max_acceleration_scaling_factor = 0.25
    plan.start_state.is_diff = True
    goal = Constraints()
    for name, position in GOALS[group].items():
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = position
        joint.tolerance_above = 0.08
        joint.tolerance_below = 0.08
        joint.weight = 1.0
        goal.joint_constraints.append(joint)
    plan.goal_constraints.append(goal)
    return request


def main():
    parser = argparse.ArgumentParser(description="Plan Gento Skye and print the trajectory.")
    parser.add_argument("--group", choices=tuple(GOALS), default="both_arms")
    parser.add_argument("--planner", choices=PLANNERS, default="RRTConnect")
    parser.add_argument("--time", type=float, default=5.0, help="seconds the planner may run")
    args = parser.parse_args()

    rclpy.init()
    node = Node("plan_gento_skye")
    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=10.0):
        raise SystemExit("move_group is not running. Launch demo.launch.py first.")

    future = client.call_async(build_request(args.group, args.planner, args.time))
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.time + 15.0)
    response = future.result()
    if response is None:
        raise SystemExit("planning call timed out")

    code = response.motion_plan_response.error_code.val
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    print(f"group={args.group}")
    print(f"planner={args.planner}")
    print(f"error_code={code}")
    print(f"points={len(trajectory.points)}")
    print(f"joints={list(trajectory.joint_names)}")
    if trajectory.points:
        last = trajectory.points[-1]
        duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        print(f"duration_sec={duration:.3f}")
        print(f"last_positions={[round(value, 3) for value in last.positions]}")
    if code != 1 or len(trajectory.points) < 2:
        raise SystemExit(f"planning failed, error_code={code}")

    shown = DisplayTrajectory()
    shown.model_id = "Gento Skye urdf"
    shown.trajectory_start = response.motion_plan_response.trajectory_start
    shown.trajectory.append(response.motion_plan_response.trajectory)
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
    publisher.publish(shown)
    rclpy.spin_once(node, timeout_sec=0.2)
    print("displayed=/display_planned_path")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
