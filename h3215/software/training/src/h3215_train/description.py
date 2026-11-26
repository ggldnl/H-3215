"""Generates the ROS description of the robot from its MJCF model.

    python -m h3215_train.description           write the files below
    python -m h3215_train.description --check   fail if they differ from what the model gives

Writes, next to the model in h3215/hardware/h3215_description/:
    urdf/h3215.urdf.xacro      links, joints, inertias, primitive shapes, a frame per site,
                               and the ros2_control block with one servo per joint
    config/joints.yaml         the joints of the leg controller and of h3215_core's posture
                               node, with the home pose

Run it after every change to the model and commit the results: the robot never needs MuJoCo.
Servo calibration (zero offset, direction) is not generated: it comes from
h3215_description/config/calibration.yaml, or the file passed as calibration_file.
"""

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

from . import morphology

OUT_DIR = morphology.DESCRIPTION
URDF_PATH = OUT_DIR / "urdf" / "h3215.urdf.xacro"
JOINTS_PATH = OUT_DIR / "config" / "joints.yaml"

# Feetech STS3215, 12 V version: URDF joint limits and the effort estimate from the load reading
STALL_TORQUE = 2.94  # N*m, 30 kg*cm
NO_LOAD_SPEED = 4.7  # rad/s, 0.222 s/60 deg
MOCK_VOLTAGE = 11.4  # V, a 3S pack half charged
MOCK_TEMPERATURE = 30.0  # degrees C


def generate(model_path: Path = morphology.DEFAULT_MODEL) -> dict[Path, str]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    robot = morphology.from_model(model)
    source = model_path.resolve().relative_to(morphology.REPO_ROOT)
    return {URDF_PATH: _urdf(model, robot, source), JOINTS_PATH: _joints_yaml(robot, source)}


def _urdf(model, robot, source):
    obj = mujoco.mjtObj
    lines = [
        '<?xml version="1.0"?>',
        f"<!-- Generated from {source} by `python -m h3215_train.description`: regenerate, don't edit -->",
        '<robot name="h3215" xmlns:xacro="http://www.ros.org/wiki/xacro">',
        '  <xacro:arg name="use_mock_hardware" default="false"/>',
        '  <xacro:arg name="port" default="/dev/ttyACM0"/>',
        '  <xacro:arg name="calibration_file" default="$(find h3215_description)/config/calibration.yaml"/>',
        "  <xacro:property name=\"calibration\" value=\"${xacro.load_yaml('$(arg calibration_file)')['joints']}\"/>",
        "",
        "  <!-- A servo's calibration: offset and direction from calibration_file, or 0 and 1 -->",
        '  <xacro:macro name="calibration_params" params="joint">',
        '    <xacro:property name="c" value="${calibration[joint] if joint in calibration else dict()}"/>',
        "    <param name=\"offset\">${c['offset'] if 'offset' in c else 0.0}</param>",
        "    <param name=\"direction\">${c['direction'] if 'direction' in c else 1}</param>",
        "  </xacro:macro>",
        "",
    ]

    # A massless root, as ROS expects (REP 105): KDL ignores the inertia of a root link
    root = mujoco.mj_name2id(model, obj.mjOBJ_BODY, morphology.BASE_BODY)
    lines += [
        '  <link name="base_link"/>',
        f'  <joint name="{morphology.BASE_BODY}_fixed" type="fixed">',
        f'    <parent link="base_link"/><child link="{morphology.BASE_BODY}"/>',
        "  </joint>",
        "",
    ]
    for b in _subtree(model, root):
        bname = mujoco.mj_id2name(model, obj.mjOBJ_BODY, b)
        lines.append(f'  <link name="{bname}">')
        if model.body_mass[b] > 0:
            i = model.body_inertia[b]
            lines += [
                "    <inertial>",
                f"      {_origin(model.body_ipos[b], model.body_iquat[b])}",
                f'      <mass value="{_f(model.body_mass[b])}"/>',
                f'      <inertia ixx="{_f(i[0])}" ixy="0" ixz="0" iyy="{_f(i[1])}" iyz="0" izz="{_f(i[2])}"/>',
                "    </inertial>",
            ]
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == b:
                lines += _visual(model, g)
        lines.append("  </link>")

        if b != root:
            parent = mujoco.mj_id2name(model, obj.mjOBJ_BODY, model.body_parentid[b])
            origin = _origin(model.body_pos[b], model.body_quat[b])
            if model.body_jntnum[b] == 0:
                lines += [
                    f'  <joint name="{bname}_fixed" type="fixed">',
                    f'    <parent link="{parent}"/><child link="{bname}"/>',
                    f"    {origin}",
                    "  </joint>",
                ]
            else:
                j = model.body_jntadr[b]
                jname = mujoco.mj_id2name(model, obj.mjOBJ_JOINT, j)
                if model.body_jntnum[b] > 1 or not np.allclose(model.jnt_pos[j], 0):
                    sys.exit(f'joint "{jname}": URDF needs one joint per body, at the body origin')
                kind = (
                    "revolute" if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE else "prismatic"
                )
                lower, upper = robot.ranges[jname]
                lines += [
                    f'  <joint name="{jname}" type="{kind}">',
                    f'    <parent link="{parent}"/><child link="{bname}"/>',
                    f"    {origin}",
                    f'    <axis xyz="{_v(model.jnt_axis[j])}"/>',
                    f'    <limit lower="{_f(lower)}" upper="{_f(upper)}" effort="{STALL_TORQUE}" velocity="{NO_LOAD_SPEED}"/>',
                    "  </joint>",
                ]
        lines.append("")

    # Sites become frames: feet for kinematics, the IMU for its messages
    for s in range(model.nsite):
        sname = mujoco.mj_id2name(model, obj.mjOBJ_SITE, s)
        if not sname:
            continue
        parent = mujoco.mj_id2name(model, obj.mjOBJ_BODY, model.site_bodyid[s])
        lines += [
            f'  <link name="{sname}"/>',
            f'  <joint name="{sname}_fixed" type="fixed">',
            f'    <parent link="{parent}"/><child link="{sname}"/>',
            f"    {_origin(model.site_pos[s], model.site_quat[s])}",
            "  </joint>",
        ]
    lines.append("")
    lines += _ros2_control(robot)
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def _ros2_control(robot):
    lines = [
        '  <ros2_control name="h3215" type="system">',
        "    <hardware>",
        '      <xacro:if value="$(arg use_mock_hardware)">',
        "        <plugin>mock_components/GenericSystem</plugin>",
        "      </xacro:if>",
        '      <xacro:unless value="$(arg use_mock_hardware)">',
        "        <plugin>h3215_hardware/StsSystem</plugin>",
        '        <param name="port">$(arg port)</param>',
        '        <param name="baudrate">1000000</param>',
        f'        <param name="stall_torque">{STALL_TORQUE}</param>',
        "      </xacro:unless>",
        "    </hardware>",
    ]
    for joint, servo_id in robot.servo_ids.items():
        lines += [
            f'    <joint name="{joint}">',
            '      <command_interface name="position"/>',
            '      <state_interface name="position">',
            f'        <param name="initial_value">{_f(robot.home[joint])}</param>',
            "      </state_interface>",
            '      <state_interface name="velocity"/>',
            '      <state_interface name="effort"/>',
            # Read by h3215_core's monitor; the initial values are what mock hardware reports
            '      <state_interface name="voltage">',
            f'        <param name="initial_value">{MOCK_VOLTAGE}</param>',
            "      </state_interface>",
            '      <state_interface name="temperature">',
            f'        <param name="initial_value">{MOCK_TEMPERATURE}</param>',
            "      </state_interface>",
            '      <state_interface name="status"/>',
            f'      <param name="id">{servo_id}</param>',
            f'      <xacro:calibration_params joint="{joint}"/>',
            "    </joint>",
        ]
    lines.append("  </ros2_control>")
    return lines


def _joints_yaml(robot, source):
    def rows(values):
        return "\n".join(f"      - {v}" for v in values)

    return (
        f"# Generated from {source} by `python -m h3215_train.description`: regenerate, don't edit\n"
        f"# {len(robot.legs)} legs, base {robot.base_height:.4f} m above the ground in the home pose\n"
        "\n"
        "leg_position_controller:\n"
        "  ros__parameters:\n"
        "    joints:\n"
        f"{rows(robot.leg_joints)}\n"
        "\n"
        "# The home pose, from the model's keyframe, in the same joint order\n"
        "posture:\n"
        "  ros__parameters:\n"
        "    joints:\n"
        f"{rows(robot.leg_joints)}\n"
        "    home:\n"
        f"{rows(repr(round(robot.home[j], 6)) for j in robot.leg_joints)}\n"
    )


def _visual(model, g):
    gtype, size = model.geom_type[g], model.geom_size[g]
    t = mujoco.mjtGeom
    if gtype == t.mjGEOM_BOX:
        shape = f'<box size="{_v(2 * size)}"/>'
    elif gtype == t.mjGEOM_SPHERE:
        shape = f'<sphere radius="{_f(size[0])}"/>'
    elif gtype in (t.mjGEOM_CAPSULE, t.mjGEOM_CYLINDER):
        # URDF has no capsule: a cylinder as long as its straight part
        shape = f'<cylinder radius="{_f(size[0])}" length="{_f(2 * size[1])}"/>'
    else:
        return [f"    <!-- geom {g}: type {gtype} not exported yet -->"]
    return [
        "    <visual>",
        f"      {_origin(model.geom_pos[g], model.geom_quat[g])}",
        f"      <geometry>{shape}</geometry>",
        f'      <material name=""><color rgba="{_v(model.geom_rgba[g])}"/></material>',
        "    </visual>",
    ]


def _subtree(model, root):
    """Bodies of the subtree in depth-first order (the order of the MJCF)."""
    return [b for b in range(model.nbody) if b == root or _descends(model, b, root)]


def _descends(model, b, root):
    while b > 0:
        b = model.body_parentid[b]
        if b == root:
            return True
    return False


def _origin(pos, quat):
    return f'<origin xyz="{_v(pos)}" rpy="{_v(_rpy(quat))}"/>'


def _rpy(quat):
    """MuJoCo quaternion (w, x, y, z) -> URDF roll, pitch, yaw (R = Rz Ry Rx)."""
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(quat, dtype=float))
    r = m.reshape(3, 3)
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])


def _f(x):
    s = f"{float(x):.6g}"
    return "0" if s in ("-0", "0") else s


def _v(values):
    return " ".join(_f(x) for x in values)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--model", type=Path, default=morphology.DEFAULT_MODEL, help="MJCF model")
    ap.add_argument("--check", action="store_true", help="only compare, write nothing")
    args = ap.parse_args()

    stale = []
    for path, text in generate(args.model).items():
        rel = path.relative_to(morphology.REPO_ROOT)
        if path.exists() and path.read_text() == text:
            continue
        stale.append(rel)
        if not args.check:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
            print(f"wrote {rel}")
    if args.check and stale:
        sys.exit("out of date, run `python -m h3215_train.description`: " + ", ".join(map(str, stale)))
    if not stale:
        print("up to date")


if __name__ == "__main__":
    main()
