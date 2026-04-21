from setuptools import find_packages, setup

package_name = 'project_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/launch',
            [
                'launch/nav_test.launch.py',
                'launch/nav_with_scan.launch.py',
                'launch/nav_with_fake_scan.launch.py',
                'launch/goal_nav_with_fake_inputs.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spectre',
    maintainer_email='spectre@todo.todo',
    description='Reactive navigation test package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reactive_nav_node = project_nav.reactive_nav_node:main',
            'fake_perception_node = project_nav.fake_perception_node:main',
            'sector_analyzer_node = project_nav.sector_analyzer_node:main',
            'fake_scan_publisher_node = project_nav.fake_scan_publisher_node:main',
            'fake_target_publisher_node = project_nav.fake_target_publisher_node:main',
            'goal_aware_nav_node = project_nav.goal_aware_nav_node:main',
        ],
    },
)