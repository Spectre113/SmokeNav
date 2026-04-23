from glob import glob

from setuptools import setup

package_name = "project_sim"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/sim_bringup.launch.py",
                "launch/sim_with_nav.launch.py",
                "launch/sim_with_nav_and_detection.launch.py",
                "launch/sim_with_smoke.launch.py",
                "launch/scenario_clear.launch.py",
                "launch/scenario_moderate.launch.py",
                "launch/scenario_dense.launch.py",
            ],
        ),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/worlds/custom-flat", glob("worlds/custom-flat/*")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spectre",
    maintainer_email="spectre@todo.todo",
    description="Gazebo Classic simulation bringup for SmokeNav",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
)

