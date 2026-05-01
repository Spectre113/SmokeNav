from setuptools import find_packages, setup

package_name = "project_smoke"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/smoke_filter.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spectre",
    maintainer_email="spectre@todo.todo",
    description="Sensor degradation node (smoke density) for simulation",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "scan_smoke_filter = project_smoke.scan_smoke_filter:main",
        ],
    },
)

