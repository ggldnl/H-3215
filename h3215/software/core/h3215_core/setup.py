from setuptools import find_packages, setup

package_name = "h3215_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Gigliotti",
    maintainer_email="ggldnl@users.noreply.github.com",
    description="H-3215's high-level interface: posture and monitor nodes",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "posture = h3215_core.posture:main",
            "monitor = h3215_core.monitor:main",
        ],
    },
)
