#!/usr/bin/env python3
"""Turn the SolidWorks Gento Skye URDF into files MoveIt can load."""

import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parent
SRC = ROOT.parent / "Gento Skye urdf(gripper)"
URDF_IN = SRC / "Gento Skye urdf" / "urdf" / "Gento Skye urdf.urdf"
MESH_DIRS = {
    "Gento Skye urdf": SRC / "Gento Skye urdf" / "meshes",
    "Pro-m6-696": SRC / "Pro-m6-696" / "meshes",
}

# Covers, wheels and the 39 MB base mesh stay visible, but are not used for collision.
DROP_COLLISION = {
    "base_link",
    "ZQ_Link",
    "ZQL_Link",
    "YQ_Link",
    "YQL_Link",
    "ZH_Link",
    "ZHL_Link",
    "YH_Link",
    "YHL_Link",
    "ZLD_Link",
    "YLD_Link",
    "IMU_Link",
    "XJ_Link",
}
DROP_COLLISION.update(f"C{i}_Link" for i in range(1, 9))

# Chain base is the parent link. MoveIt does not put that link's own parent joint in the group.
ARM_CHAINS = {
    "right_arm": ("waist_Link_3", "TCP_Link_R"),
    "left_arm": ("waist_Link_3", "TCP_Link_L"),
}
GRIPPER_JOINTS = {
    "right_gripper": ["Arm_R10_Joint", "Arm_R11_Joint"],
    "left_gripper": ["Arm_L10_Joint", "Arm_L11_Joint"],
}
OTHER_GROUPS = {
    "waist": ["waist_jiont_1_prismatic", "waist_joint_2", "waist_joint_3"],
    "head": ["head_joint_1", "head_joint_2"],
}


def file_uri(path: Path) -> str:
    return "file://" + quote(str(path.resolve()), safe="/")


def mesh_path(filename: str) -> Path:
    prefix = "package://"
    if not filename.startswith(prefix):
        raise SystemExit(f"unexpected mesh uri: {filename}")
    rest = filename[len(prefix):]
    package, _, relative = rest.partition("/")
    if package not in MESH_DIRS or not relative.startswith("meshes/"):
        raise SystemExit(f"unexpected mesh uri: {filename}")
    name = relative[len("meshes/"):]
    path = MESH_DIRS[package] / name
    if path.is_file():
        return path
    # The Skye URDF points TCP_Link_R at Pro-m6-696, but that STL only exists on the Skye mesh set.
    for fallback in MESH_DIRS.values():
        candidate = fallback / name
        if candidate.is_file():
            return candidate
    raise SystemExit(f"missing mesh: {path}")


def main():
    tree = ET.parse(URDF_IN)
    root = tree.getroot()
    limits = {joint.attrib["name"]: joint.find("limit") for joint in root.findall("joint")}
    # The Skye export writes Joint1_L as [0, 0]. The arm URDF in the same folder has the real range.
    left = limits["Joint1_L"]
    right = limits["Joint1_R"]
    left.attrib["lower"] = right.attrib["lower"]
    left.attrib["upper"] = right.attrib["upper"]
    left.attrib["effort"] = right.attrib.get("effort", "0")
    left.attrib["velocity"] = right.attrib.get("velocity", "1")

    for link in root.findall("link"):
        name = link.attrib["name"]
        if "camera" in name or name in DROP_COLLISION:
            for collision in list(link.findall("collision")):
                link.remove(collision)
        inertial = link.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            if mass is not None and float(mass.attrib.get("value", "1")) == 0.0:
                mass.attrib["value"] = "0.001"
                inertia = inertial.find("inertia")
                if inertia is not None:
                    for key in ("ixx", "iyy", "izz"):
                        if float(inertia.attrib.get(key, "0")) == 0.0:
                            inertia.attrib[key] = "1e-6"
        for mesh in link.iter("mesh"):
            mesh.attrib["filename"] = file_uri(mesh_path(mesh.attrib["filename"]))

    urdf_out = ROOT / "config" / "gento_skye.urdf"
    tree.write(urdf_out, encoding="utf-8", xml_declaration=True)

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<robot name="{root.attrib["name"]}">',
        '  <virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base_link"/>',
    ]
    for group, (base, tip) in ARM_CHAINS.items():
        lines.append(f'  <group name="{group}">')
        lines.append(f'    <chain base_link="{base}" tip_link="{tip}"/>')
        lines.append("  </group>")
    lines.append('  <group name="both_arms">')
    lines.append('    <group name="left_arm"/>')
    lines.append('    <group name="right_arm"/>')
    lines.append("  </group>")
    for group, names in {**GRIPPER_JOINTS, **OTHER_GROUPS}.items():
        lines.append(f'  <group name="{group}">')
        for name in names:
            lines.append(f'    <joint name="{name}"/>')
        lines.append("  </group>")
    lines.append('  <end_effector name="right_gripper" parent_link="Link8_R" group="right_gripper"/>')
    lines.append('  <end_effector name="left_gripper" parent_link="Link8_L" group="left_gripper"/>')
    parent_of = {}
    for joint in root.findall("joint"):
        parent_of[joint.find("child").attrib["link"]] = joint.find("parent").attrib["link"]
    # Wrist meshes overlap their grandparent. Adjacent pairs alone still report Link7 vs Link8.
    pairs = set()
    for link in parent_of:
        cursor = link
        for _ in range(3):
            cursor = parent_of.get(cursor)
            if cursor is None:
                break
            pairs.add(tuple(sorted((link, cursor))))
    for link_a, link_b in sorted(pairs):
        lines.append(
            f'  <disable_collisions link1="{link_a}" link2="{link_b}" reason="Adjacent"/>'
        )
    lines.append("</robot>")
    (ROOT / "config" / "gento_skye.srdf").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {urdf_out}")
    print("Joint1_L limits", left.attrib["lower"], left.attrib["upper"])


if __name__ == "__main__":
    main()
