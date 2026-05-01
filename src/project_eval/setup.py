from setuptools import find_packages, setup

package_name = "project_eval"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spectre",
    maintainer_email="spectre@todo.todo",
    description="Lightweight metrics logging for simulation runs",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "metrics_logger = project_eval.metrics_logger:main",
        ],
    },
)

