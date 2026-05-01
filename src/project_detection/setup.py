from setuptools import find_packages, setup

package_name = "project_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/detection_from_gazebo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spectre",
    maintainer_email="spectre@todo.todo",
    description="Simple human detection output from Gazebo model states",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "gazebo_human_detector = project_detection.gazebo_human_detector:main",
        ],
    },
)

