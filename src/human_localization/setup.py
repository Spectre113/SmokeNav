from setuptools import setup

package_name = 'human_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='an.krasnov@innopolis.university',
    description='Human localization module',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'human_localization = human_localization.human_localization_node:main',
            'human_pose_adapter = human_localization.human_pose_adapter_node:main',
        ],
    },
)
