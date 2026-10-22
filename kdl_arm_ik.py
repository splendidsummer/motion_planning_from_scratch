#!/usr/bin/python3
"""Initial 7-DOF KDL inverse kinematics for the Gento Skye arms.

Each arm has seven revolute joints and a 6-D TCP pose, so one pose is a curve
of solutions. The waist stays locked. By default both arms are solved, and the
more applicable arm is kept: elbow in front, then the shorter joint-space reach
from that arm's seed. This is the same choice scene_gento_skye.choose_arm()
makes after MoveIt's KDL service.

One seed uses ChainIkSolverPos_LMA with equal weights on position and rotation.
LMA adapts its damping, and its constructor does not take joint limits. When
that pose lies outside the URDF limits, the same seed is solved again with
ChainIkSolverPos_NR_JL and ChainIkSolverVel_wdls, which clamps the joints on
every step. Elbow and arm choice stay outside both solvers.

Collision with the table and the scene is not checked here. Run this with the
system Python; the conda interpreter does not have PyKDL:

    /usr/bin/python3 kdl_arm_ik.py --self-test
    /usr/bin/python3 kdl_arm_ik.py \\
        --xyz 0.70 -0.18 0.52 --quat 0.31100 0.65408 0.40126 0.56075
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import PyKDL

ROOT = Path(__file__).resolve().parent
URDF_PATH = ROOT / "gento_skye_moveit_config" / "config" / "gento_skye.urdf"
CHAIN_BASE = "waist_Link_3"
WAIST_NAMES = ("waist_jiont_1_prismatic", "waist_joint_2", "waist_joint_3")
ARMS = {
    "right": {"tip": "TCP_Link_R", "elbow": "Link4_R", "side": -1.0},
    "left": {"tip": "TCP_Link_L", "elbow": "Link4_L", "side": 1.0},
}
POSITION_TOLERANCE_M = 1e-4
ROTATION_TOLERANCE_RAD = 1e-4
# LMA's default task weight is diag(1, 1, 1, 0.01, 0.01, 0.01), which nearly
# ignores orientation. A grasp locks the full pose, so the six weights match.
LMA_TASK_WEIGHTS = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
LMA_MAX_ITERATIONS = 500
# NR_JL clamps to these limits. WDLS damping stays fixed for that second solve.
WDLS_DAMPING = 1e-2
NR_MAX_ITERATIONS = 200
# Same geometric rule as elbow_in_front() in docs/scene_gento_skye.py.
ELBOW_MIN_X = 0.25
ELBOW_MAX_Z = 1.05
ELBOW_MIN_SIDE = -0.05
# choose_arm() keeps the shorter reach. Inside this band the arm on the
# target's side of the body is the more applicable one.
DISTANCE_TIE_RAD = 0.25
SIDE_Y_M = 0.05
# Home leaves both hands out to the side. These are extra seeds that bend the
# elbow and pitch the upper arm forward. They are search starts, not a calibrated pose.
REACH_SEEDS = {
    "right": (
        (-0.6, -1.0, 0.0, -1.4, 0.0, 0.6, 0.0),
        (-0.6, -1.4, 0.4, -0.9, 0.0, 0.4, 0.0),
    ),
    "left": (
        (0.6, -1.0, 0.0, -1.4, 0.0, 0.6, 0.0),
        (0.0, -1.4, 1.0, -0.8, 0.0, -0.6, 0.0),
    ),
}


def parse_floats(text):
    return tuple(float(part) for part in text.split())


def joint_origin(joint):
    origin = joint.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return (
        parse_floats(origin.attrib.get("xyz", "0 0 0")),
        parse_floats(origin.attrib.get("rpy", "0 0 0")),
    )


def joint_axis(joint):
    axis = joint.find("axis")
    if axis is None:
        return (0.0, 0.0, 1.0)
    return parse_floats(axis.attrib.get("xyz", "0 0 1"))


def joint_limits(joint):
    kind = joint.attrib["type"]
    if kind == "fixed":
        return None
    if kind not in ("revolute", "prismatic"):
        raise SystemExit(f"unsupported joint {joint.attrib['name']} type {kind}")
    limit = joint.find("limit")
    if limit is None:
        raise SystemExit(f"moving joint {joint.attrib['name']} has no limit")
    return float(limit.attrib["lower"]), float(limit.attrib["upper"])


def load_model(path=URDF_PATH):
    root = ET.parse(path).getroot()
    by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child").attrib["link"]
        if child in by_child:
            raise SystemExit(f"two joints claim child link {child}")
        by_child[child] = joint
    return by_child


def path_joints(by_child, tip, base):
    joints = []
    link = tip
    seen = set()
    while link != base:
        if link in seen:
            raise SystemExit(f"cycle while walking to {base} at {link}")
        seen.add(link)
        joint = by_child.get(link)
        if joint is None:
            raise SystemExit(f"no parent joint for link {link}")
        joints.append(joint)
        link = joint.find("parent").attrib["link"]
    joints.reverse()
    return joints


def kdl_segment(joint):
    """Match ros2 kdl_parser: axis is expressed in the parent frame."""
    name = joint.attrib["name"]
    xyz, rpy = joint_origin(joint)
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
    kind = joint.attrib["type"]
    child = joint.find("child").attrib["link"]
    if kind == "fixed":
        kdl_joint = PyKDL.Joint(name, PyKDL.Joint.Fixed)
    else:
        axis = frame.M * PyKDL.Vector(*joint_axis(joint))
        jtype = PyKDL.Joint.RotAxis if kind == "revolute" else PyKDL.Joint.TransAxis
        kdl_joint = PyKDL.Joint(name, frame.p, axis, jtype)
    return PyKDL.Segment(child, kdl_joint, frame)


def build_chain(joints):
    chain = PyKDL.Chain()
    for joint in joints:
        chain.addSegment(kdl_segment(joint))
    return chain


def moving_joints(joints):
    names = []
    lower = []
    upper = []
    for joint in joints:
        limits = joint_limits(joint)
        if limits is None:
            continue
        names.append(joint.attrib["name"])
        lower.append(limits[0])
        upper.append(limits[1])
    return names, lower, upper


def list_to_jnt(values):
    array = PyKDL.JntArray(len(values))
    for index, value in enumerate(values):
        array[index] = float(value)
    return array


def jnt_to_list(array):
    return [float(array[index]) for index in range(array.rows())]


def frame_to_matrix(frame):
    matrix = np.eye(4)
    for row in range(3):
        for col in range(3):
            matrix[row, col] = frame.M[row, col]
        matrix[row, 3] = frame.p[row]
    return matrix


def fk_frame(chain, values):
    solver = PyKDL.ChainFkSolverPos_recursive(chain)
    frame = PyKDL.Frame()
    code = solver.JntToCart(list_to_jnt(values), frame)
    if code < 0:
        raise SystemExit("KDL forward kinematics failed")
    return frame


def twist_error(current, target):
    twist = PyKDL.diff(current, target)
    return np.array(
        [
            twist.vel.x(),
            twist.vel.y(),
            twist.vel.z(),
            twist.rot.x(),
            twist.rot.y(),
            twist.rot.z(),
        ],
        dtype=float,
    )


def error_norms(error):
    return float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))


def clip_joints(values, lower, upper):
    return [min(upper[i], max(lower[i], values[i])) for i in range(len(values))]


def inside_limits(values, lower, upper):
    return all(
        math.isfinite(value) and lower[i] - 1e-8 <= value <= upper[i] + 1e-8
        for i, value in enumerate(values)
    )


def pose_of(chain, values, target):
    if not all(math.isfinite(value) for value in values):
        return None
    try:
        frame = fk_frame(chain, values)
    except SystemExit:
        return None
    position, rotation = error_norms(twist_error(frame, target))
    if not math.isfinite(position) or not math.isfinite(rotation):
        return None
    return position, rotation


def pose_reached(position, rotation):
    return position < POSITION_TOLERANCE_M and rotation < ROTATION_TOLERANCE_RAD


def run_lma(chain, seed, target):
    solver = PyKDL.ChainIkSolverPos_LMA(
        chain,
        list(LMA_TASK_WEIGHTS),
        1e-12,
        LMA_MAX_ITERATIONS,
        1e-15,
    )
    output = PyKDL.JntArray(chain.getNrOfJoints())
    solver.CartToJnt(list_to_jnt(seed), target, output)
    return jnt_to_list(output)


def run_limited(chain, seed, lower, upper, target):
    """Damped steps that clamp into the URDF limits after every update."""
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    velocity = PyKDL.ChainIkSolverVel_wdls(chain)
    velocity.setLambda(WDLS_DAMPING)
    solver = PyKDL.ChainIkSolverPos_NR_JL(
        chain,
        list_to_jnt(lower),
        list_to_jnt(upper),
        fk_solver,
        velocity,
        NR_MAX_ITERATIONS,
        1e-6,
    )
    output = PyKDL.JntArray(chain.getNrOfJoints())
    solver.CartToJnt(list_to_jnt(seed), target, output)
    return jnt_to_list(output)


def solve_seed(chain, seed, nominal, lower, upper, target):
    """Pose with LMA. A second, limit-clamped solve only if that pose is illegal.

    ``nominal`` is the same start the older null-space pull used. LMA and WDLS
    do not apply that pull; the caller still ranks finished poses by distance
    to the original seed.
    """
    del nominal
    start = clip_joints(seed, lower, upper)
    attempts = []
    lma_values = run_lma(chain, start, target)
    lma_pose = pose_of(chain, lma_values, target)
    if lma_pose is not None:
        attempts.append((lma_values, lma_pose[0], lma_pose[1]))
        if pose_reached(*lma_pose) and inside_limits(lma_values, lower, upper):
            return attempts[-1]
    limited_values = run_limited(chain, start, lower, upper, target)
    limited_pose = pose_of(chain, limited_values, target)
    if limited_pose is not None and inside_limits(limited_values, lower, upper):
        attempts.append((limited_values, limited_pose[0], limited_pose[1]))
        if pose_reached(*limited_pose):
            return attempts[-1]
    legal = [item for item in attempts if inside_limits(item[0], lower, upper)]
    pool = legal or attempts
    if not pool:
        return None
    pool.sort(key=lambda item: item[1] + item[2])
    return pool[0]


def target_in_waist(waist_frame, xyz, rotation):
    base_target = PyKDL.Frame(rotation, PyKDL.Vector(*xyz))
    return waist_frame.Inverse() * base_target


def rotation_from_args(quat, rpy):
    if quat is not None and rpy is not None:
        raise SystemExit("pass --quat or --rpy, not both")
    if rpy is not None:
        return PyKDL.Rotation.RPY(*rpy)
    if quat is None:
        raise SystemExit("a TCP orientation is required: --quat x y z w or --rpy")
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise SystemExit("quaternion has zero length")
    return PyKDL.Rotation.Quaternion(x / norm, y / norm, z / norm, w / norm)


def elbow_position(elbow_chain, waist_frame, arm_values):
    count = elbow_chain.getNrOfJoints()
    local = fk_frame(elbow_chain, arm_values[:count])
    point = waist_frame * local.p
    return (point.x(), point.y(), point.z())


def elbow_in_front(position, side):
    x, y, z = position
    return x > ELBOW_MIN_X and z < ELBOW_MAX_Z and y * side > ELBOW_MIN_SIDE


def joint_distance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def limit_margin(values, lower, upper):
    return min(min(value - lo, hi - value) for value, lo, hi in zip(values, lower, upper))


def seed_family(reference, lower, upper, arm):
    """Reference, a bent elbow, a Joint3 sweep, then forward-reach starts."""
    reference = clip_joints(list(reference), lower, upper)
    bent = list(reference)
    if abs(bent[3]) < 0.4:
        bent[3] = -0.9
    bent = clip_joints(bent, lower, upper)
    family = [reference, bent]
    for swivel in (-1.2, -0.6, 0.6, 1.2):
        varied = list(bent)
        varied[2] = swivel
        family.append(clip_joints(varied, lower, upper))
    for reach in REACH_SEEDS[arm]:
        family.append(clip_joints(reach, lower, upper))
    unique = []
    for values in family:
        if not any(joint_distance(values, kept) < 1e-6 for kept in unique):
            unique.append(values)
    return unique


def solve_arm(model, arm, xyz, rotation, seed, waist, require_elbow):
    spec = ARMS[arm]
    waist_joints = path_joints(model, CHAIN_BASE, "base_link")
    arm_joints = path_joints(model, spec["tip"], CHAIN_BASE)
    elbow_joints = path_joints(model, spec["elbow"], CHAIN_BASE)
    waist_names, waist_lower, waist_upper = moving_joints(waist_joints)
    arm_names, arm_lower, arm_upper = moving_joints(arm_joints)
    if tuple(waist_names) != WAIST_NAMES:
        raise SystemExit(f"unexpected waist joints: {waist_names}")
    if len(arm_names) != 7 or len(seed) != 7 or len(waist) != 3:
        raise SystemExit("this solver expects 3 locked waist joints and 7 arm joints")
    for index, value in enumerate(waist):
        if value < waist_lower[index] - 1e-9 or value > waist_upper[index] + 1e-9:
            raise SystemExit(
                f"{waist_names[index]}={value} is outside [{waist_lower[index]}, {waist_upper[index]}]"
            )
    waist_chain = build_chain(waist_joints)
    arm_chain = build_chain(arm_joints)
    elbow_chain = build_chain(elbow_joints)
    if arm_chain.getNrOfJoints() != 7 or elbow_chain.getNrOfJoints() != 4:
        raise SystemExit("arm chain lost or gained a moving joint")
    waist_frame = fk_frame(waist_chain, waist)
    local_target = target_in_waist(waist_frame, xyz, rotation)
    found = []
    miss = None
    for index, guess in enumerate(seed_family(seed, arm_lower, arm_upper, arm)):
        solved = solve_seed(arm_chain, guess, guess, arm_lower, arm_upper, local_target)
        if solved is None:
            continue
        values, position_error, rotation_error = solved
        if position_error > POSITION_TOLERANCE_M or rotation_error > ROTATION_TOLERANCE_RAD:
            cost = position_error + rotation_error
            if miss is None or cost < miss["cost"]:
                miss = {
                    "cost": cost,
                    "position_error": position_error,
                    "rotation_error": rotation_error,
                }
            continue
        values = clip_joints(values, arm_lower, arm_upper)
        elbow = elbow_position(elbow_chain, waist_frame, values)
        found.append(
            {
                "values": values,
                "position_error": position_error,
                "rotation_error": rotation_error,
                "distance": joint_distance(values, seed),
                "margin": limit_margin(values, arm_lower, arm_upper),
                "elbow": elbow,
                "elbow_ok": elbow_in_front(elbow, spec["side"]),
                "seed_index": index,
            }
        )
    if not found:
        return None, miss, arm_names
    eligible = [item for item in found if item["elbow_ok"]] if require_elbow else found
    pool = eligible or found
    pool.sort(key=lambda item: (item["distance"], -item["margin"]))
    chosen = pool[0]
    chosen["names"] = arm_names
    chosen["elbow_required"] = require_elbow
    chosen["rejected_for_elbow"] = bool(require_elbow and not eligible)
    achieved = waist_frame * fk_frame(arm_chain, chosen["values"])
    chosen["achieved_xyz"] = (achieved.p.x(), achieved.p.y(), achieved.p.z())
    chosen["achieved_rpy"] = achieved.M.GetRPY()
    return chosen, None, arm_names


def arm_for_target_side(y):
    if y < -SIDE_Y_M:
        return "right"
    if y > SIDE_Y_M:
        return "left"
    return None


def select_arm(results, y, require_elbow):
    """Pick the applicable arm from both KDL solutions.

    An applicable solution has reached the pose. Elbow-in-front wins over an
    elbow-behind solution. Otherwise the shorter distance to that arm's seed
    wins, matching choose_arm(). When the two distances are within
    DISTANCE_TIE_RAD and the target is off the center line, the arm on that
    side is kept.
    """
    converged = []
    for arm in ("right", "left"):
        solution, _miss, _names = results[arm]
        if solution is not None:
            converged.append((arm, solution))
    if not converged:
        return None, "none"
    front = [(arm, solution) for arm, solution in converged if solution["elbow_ok"]]
    pool = front if require_elbow and front else converged
    pool.sort(key=lambda item: (item[1]["distance"], -item[1]["margin"]))
    chosen, solution = pool[0]
    reason = "only_converged" if len(pool) == 1 else "shorter_joint_distance"
    if len(pool) > 1 and abs(pool[0][1]["distance"] - pool[1][1]["distance"]) <= DISTANCE_TIE_RAD:
        preferred = arm_for_target_side(y)
        if preferred is not None and any(arm == preferred for arm, _item in pool):
            chosen, solution = next(item for item in pool if item[0] == preferred)
            reason = "same_side"
    if require_elbow and not solution["elbow_ok"]:
        reason = "elbow_behind"
    return chosen, reason


def solve_both(model, xyz, rotation, seeds, waist, require_elbow):
    results = {}
    for arm in ("right", "left"):
        results[arm] = solve_arm(
            model, arm, xyz, rotation, seeds[arm], waist, require_elbow,
        )
    selected, reason = select_arm(results, xyz[1], require_elbow)
    return selected, reason, results


def format_solution(arm, solution, miss=None):
    lines = [
        f"arm={arm}_arm",
        f"converged={0 if solution is None else 1}",
    ]
    if solution is None:
        if miss is None:
            lines.append("position_error_m=none")
            lines.append("rotation_error_rad=none")
        else:
            lines.append(f"position_error_m={miss['position_error']:.3e}")
            lines.append(f"rotation_error_rad={miss['rotation_error']:.3e}")
        lines.append("joints=none")
        return "\n".join(lines)
    lines.extend(
        [
            f"elbow_in_front={int(solution['elbow_ok'])}",
            f"elbow_xyz={[round(value, 4) for value in solution['elbow']]}",
            f"position_error_m={solution['position_error']:.3e}",
            f"rotation_error_rad={solution['rotation_error']:.3e}",
            f"joint_distance={solution['distance']:.4f}",
            f"limit_margin_rad={solution['margin']:.4f}",
            f"seed_index={solution['seed_index']}",
            "achieved_xyz="
            + " ".join(f"{value:.6f}" for value in solution["achieved_xyz"]),
            "achieved_rpy="
            + " ".join(f"{value:.6f}" for value in solution["achieved_rpy"]),
        ]
    )
    pairs = []
    for name, value in zip(solution["names"], solution["values"]):
        lines.append(f"{name}={value:.6f}")
        pairs.append(f'"{name}": {value:.6f}')
    lines.append("joints={" + ", ".join(pairs) + "}")
    return "\n".join(lines)


def format_arm_status(arm, solution, miss):
    if solution is None:
        if miss is None:
            return f"{arm}_arm converged=0 position_error_m=none rotation_error_rad=none"
        return (
            f"{arm}_arm converged=0 "
            f"position_error_m={miss['position_error']:.3e} "
            f"rotation_error_rad={miss['rotation_error']:.3e}"
        )
    return (
        f"{arm}_arm converged=1 elbow_in_front={int(solution['elbow_ok'])} "
        f"joint_distance={solution['distance']:.4f} limit_margin_rad={solution['margin']:.4f}"
    )


def format_choice(selected, reason, results):
    lines = [
        f"selected={selected + '_arm' if selected else 'none'}",
        f"reason={reason}",
    ]
    for arm in ("right", "left"):
        solution, miss, _names = results[arm]
        lines.append(format_arm_status(arm, solution, miss))
    if selected is not None:
        lines.append(format_solution(selected, results[selected][0], results[selected][1]))
    return "\n".join(lines)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def numpy_fk(joints, values_by_name):
    """Independent URDF forward kinematics: origin, then motion in the joint frame."""
    transform = np.eye(4)
    moving = 0
    for joint in joints:
        xyz, rpy = joint_origin(joint)
        origin = np.eye(4)
        origin[:3, :3] = rpy_matrix(*rpy)
        origin[:3, 3] = xyz
        transform = transform @ origin
        kind = joint.attrib["type"]
        if kind == "fixed":
            continue
        axis = np.asarray(joint_axis(joint), dtype=float)
        value = values_by_name[joint.attrib["name"]]
        motion = np.eye(4)
        if kind == "revolute":
            angle = float(value)
            unit = axis / np.linalg.norm(axis)
            x, y, z = unit
            c, s = math.cos(angle), math.sin(angle)
            skew = 1.0 - c
            motion[:3, :3] = np.array(
                [
                    [c + x * x * skew, x * y * skew - z * s, x * z * skew + y * s],
                    [y * x * skew + z * s, c + y * y * skew, y * z * skew - x * s],
                    [z * x * skew - y * s, z * y * skew + x * s, c + z * z * skew],
                ]
            )
        else:
            motion[:3, 3] = axis * float(value)
        transform = transform @ motion
        moving += 1
    return transform, moving


def rotation_error(left, right):
    relative = left[:3, :3].T @ right[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def self_test():
    model = load_model()
    failures = []

    def check(condition, message):
        if not condition:
            failures.append(message)

    for arm, waist in (("right", (0.0, 0.0, 0.0)), ("left", (0.2, -0.4, 0.2))):
        spec = ARMS[arm]
        full = path_joints(model, spec["tip"], "base_link")
        waist_joints = path_joints(model, CHAIN_BASE, "base_link")
        arm_joints = path_joints(model, spec["tip"], CHAIN_BASE)
        waist_names, _, _ = moving_joints(waist_joints)
        arm_names, lower, upper = moving_joints(arm_joints)
        sample = [0.4, 0.5, -0.7, -1.1, 0.3, 0.45, -0.2]
        if arm == "left":
            sample = [-0.4, -0.5, 0.7, -1.1, -0.3, 0.45, 0.2]
        sample = clip_joints(sample, lower, upper)
        named = {name: value for name, value in zip(waist_names, waist)}
        named.update({name: value for name, value in zip(arm_names, sample)})
        numeric, moving = numpy_fk(full, named)
        check(moving == 10, f"{arm} full chain should have 10 moving joints, got {moving}")
        waist_frame = fk_frame(build_chain(waist_joints), waist)
        arm_frame = fk_frame(build_chain(arm_joints), sample)
        combined = frame_to_matrix(waist_frame * arm_frame)
        position_gap = float(np.linalg.norm(numeric[:3, 3] - combined[:3, 3]))
        rotation_gap = rotation_error(numeric, combined)
        check(position_gap < 1e-6, f"{arm} FK position gap {position_gap}")
        check(rotation_gap < 1e-6, f"{arm} FK rotation gap {rotation_gap}")

        true_frame = waist_frame * arm_frame
        solution, _miss, names = solve_arm(
            model,
            arm,
            (true_frame.p.x(), true_frame.p.y(), true_frame.p.z()),
            true_frame.M,
            sample,
            waist,
            require_elbow=False,
        )
        check(solution is not None, f"{arm} round trip found no solution")
        if solution is not None:
            check(names == arm_names, f"{arm} joint names changed")
            check(solution["position_error"] < POSITION_TOLERANCE_M, f"{arm} position residual")
            check(solution["rotation_error"] < ROTATION_TOLERANCE_RAD, f"{arm} rotation residual")
            distance = joint_distance(solution["values"], sample)
            check(distance < 0.35, f"{arm} drifted from the reference by {distance:.3f} rad")
            check(solution["margin"] >= -1e-8, f"{arm} solution left the joint limits")

    missing, _miss, _names = solve_arm(
        model,
        "right",
        (2.5, 0.0, 0.5),
        PyKDL.Rotation.Identity(),
        [0.0] * 7,
        (0.0, 0.0, 0.0),
        require_elbow=False,
    )
    check(missing is None, "an unreachable pose was reported as solved")

    for arm, sample in (
        ("right", [0.4, 0.5, -0.7, -1.1, 0.3, 0.45, -0.2]),
        ("left", [-0.4, -0.5, 0.7, -1.1, -0.3, 0.45, 0.2]),
    ):
        arm_joints = path_joints(model, ARMS[arm]["tip"], CHAIN_BASE)
        names, lower, upper = moving_joints(arm_joints)
        sample = clip_joints(sample, lower, upper)
        waist_frame = fk_frame(build_chain(path_joints(model, CHAIN_BASE, "base_link")), (0.0, 0.0, 0.0))
        true_frame = waist_frame * fk_frame(build_chain(arm_joints), sample)
        seeds = {"right": list(sample), "left": list(sample)}
        selected, _reason, both = solve_both(
            model,
            (true_frame.p.x(), true_frame.p.y(), true_frame.p.z()),
            true_frame.M,
            seeds,
            (0.0, 0.0, 0.0),
            require_elbow=False,
        )
        check(selected == arm, f"both-arm solve selected {selected} for a {arm} pose")
        if selected == arm:
            owned = both[arm][0]
            check(owned is not None and joint_distance(owned["values"], sample) < 0.35,
                  f"{arm} both-arm solution left its seed")

    if failures:
        for message in failures:
            print(f"FAIL {message}")
        raise SystemExit(f"self-test failed: {len(failures)}")
    print("self-test=ok")
    print("fk_agreement_m<1e-6")
    print("round_trip_position_m<1e-4")
    print("round_trip_rotation_rad<1e-4")
    print("reference_distance_rad<0.35")
    print("both_arms_select_owner")


def main():
    parser = argparse.ArgumentParser(description="Solve both Gento Skye arms with KDL and keep the applicable one.")
    parser.add_argument("--arm", choices=("both",) + tuple(ARMS), default="both")
    parser.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--quat", nargs=4, type=float, metavar=("X", "Y", "Z", "W"))
    parser.add_argument("--rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--seed", nargs=7, type=float, default=[0.0] * 7,
                        help="reference joints used for an arm without its own seed")
    parser.add_argument("--seed-right", nargs=7, type=float, default=None)
    parser.add_argument("--seed-left", nargs=7, type=float, default=None)
    parser.add_argument(
        "--waist",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        metavar=("LIFT_M", "YAW", "ROLL"),
        help="locked waist_jiont_1_prismatic, waist_joint_2, waist_joint_3",
    )
    parser.add_argument(
        "--no-elbow-front",
        action="store_true",
        help="keep the nearest pose solution even when the elbow is behind the chest",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.xyz is None:
        parser.error("--xyz is required unless --self-test is set")
    model = load_model()
    rotation = rotation_from_args(args.quat, args.rpy)
    waist = tuple(args.waist)
    require_elbow = not args.no_elbow_front
    seeds = {
        "right": list(args.seed if args.seed_right is None else args.seed_right),
        "left": list(args.seed if args.seed_left is None else args.seed_left),
    }
    if args.arm == "both":
        selected, _reason, results = solve_both(
            model, tuple(args.xyz), rotation, seeds, waist, require_elbow,
        )
        print(format_choice(selected, _reason, results))
        solution = None if selected is None else results[selected][0]
    else:
        solution, miss, _names = solve_arm(
            model, args.arm, tuple(args.xyz), rotation, seeds[args.arm], waist, require_elbow,
        )
        print(format_solution(args.arm, solution, miss))
    if solution is None:
        raise SystemExit(2)
    if solution["rejected_for_elbow"]:
        raise SystemExit(3)


if __name__ == "__main__":
    sys.exit(main())
