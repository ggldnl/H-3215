"""Checks the model parser and the generated ROS description (no hardware needed).

    uv run --project h3215/software/training python tests/test_morphology.py

Needs mujoco: run it on a PC, after `uv sync --project h3215/software/training`.
"""
import math

import mujoco

from h3215_train import description, morphology


def make_model(legs=6, joints=3, rename=None, home=True):
    """A model that follows the convention: legs around a box, each a chain of hinges."""
    bodies = []
    for i in range(1, legs + 1):
        angle = 2 * math.pi * (i - 0.5) / legs
        chain = '<site name="leg_{i}_foot" pos="0.05 0 -0.1"/><geom name="leg_{i}_foot_collision" type="sphere" size="0.01" pos="0.05 0 -0.1"/>'.format(i=i)
        for j in range(joints, 0, -1):
            axis = "0 0 1" if j == 1 else "0 -1 0"
            pos = f"{0.08 * math.cos(angle)} {0.08 * math.sin(angle)} 0" if j == 1 else "0.05 0 0"
            euler = f' euler="0 0 {angle}"' if j == 1 else ""
            chain = (
                f'<body name="leg_{i}_link_{j}" pos="{pos}"{euler}>'
                f'<joint name="leg_{i}_joint_{j}" axis="{axis}" range="-1 1"/>'
                f'<geom name="leg_{i}_link_{j}_collision" type="capsule" fromto="0 0 0 0.05 0 0" size="0.01"/>'
                f"{chain}</body>"
            )
        bodies.append(chain)
    xml = (
        '<mujoco><compiler angle="radian"/><worldbody><body name="base" pos="0 0 0.2"><freejoint/>'
        '<geom name="base_collision" type="box" size="0.08 0.06 0.02"/><site name="imu"/>'
        + "".join(bodies)
        + "</body></worldbody>"
        + (f'<keyframe><key name="home" qpos="0 0 0.2 1 0 0 0 {" 0.2" * legs * joints}"/></keyframe>' if home else "")
        + "</mujoco>"
    )
    for old, new in (rename or {}).items():
        xml = xml.replace(old, new)
    return mujoco.MjModel.from_xml_string(xml)


def expect_error(text, **kwargs):
    try:
        morphology.from_model(make_model(**kwargs))
    except morphology.ModelError as e:
        assert text in str(e), f"expected {text!r} in {e!r}"
        return
    raise AssertionError(f"no error, expected {text!r}")


def test_h3215():
    robot = morphology.load()
    assert len(robot.legs) == 6 and all(len(leg.joints) == 3 for leg in robot.legs)
    assert robot.joints[:3] == ("leg_1_joint_1", "leg_1_joint_2", "leg_1_joint_3")
    assert robot.home["leg_1_joint_2"] == 0.35 and robot.home["leg_6_joint_3"] == -0.25
    assert abs(robot.base_height - 0.117) < 0.001
    assert robot.servo_ids["leg_1_joint_1"] == 1 and robot.servo_ids["leg_6_joint_3"] == 18
    assert robot.foot_sites[0] == "leg_1_foot" and robot.imu_site == "imu"
    assert "leg_1_foot_collision" not in robot.leg_collision_geoms


def test_other_morphologies():
    for legs, joints in ((4, 3), (6, 4), (8, 2)):
        robot = morphology.from_model(make_model(legs, joints))
        assert len(robot.legs) == legs
        assert len(robot.leg_joints) == legs * joints
        assert robot.legs[-1].joints[-1] == f"leg_{legs}_joint_{joints}"
        assert robot.base_height > 0


def test_convention_errors():
    expect_error("numbered 1, 2", rename={"leg_2_joint_": "leg_7_joint_"})
    expect_error("must belong to body", rename={'name="leg_1_link_2"': 'name="leg_1_thigh"'})
    expect_error('needs site "leg_3_foot"', rename={'site name="leg_3_foot"': 'site name="leg_3_toe"'})
    expect_error("isn't leg_<i>_joint_<j>", rename={"leg_1_joint_3": "leg_1_tibia_joint"})
    expect_error('no keyframe named "home"', home=False)


def test_description_up_to_date():
    for path, text in description.generate().items():
        assert path.read_text() == text, f"{path.name} is stale: run python -m h3215_train.description"


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
    print("all tests passed")
