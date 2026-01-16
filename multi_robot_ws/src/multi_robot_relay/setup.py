from setuptools import find_packages, setup
import os

package_name = 'multi_robot_relay'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/multi_robot_control.launch.py',
             'launch/multi_robot_control_full.launch.py',
             'launch/auto_multi_robot_control.launch.py',
             'launch/dynamic_multi_robot_control.launch.py']),
        (os.path.join('share', package_name, 'config'),
            ['config/domain_bridge_robot1.yaml',
             'config/domain_bridge_robot2.yaml',
             'config/domain_bridge_robot3.yaml',
             'config/domain_bridge_robot1_full.yaml',
             'config/domain_bridge_robot2_full.yaml',
             'config/domain_bridge_robot3_full.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='region',
    maintainer_email='region@todo.todo',
    description='Multi-robot topic relay for centralized control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'topic_relay = multi_robot_relay.topic_relay:main',
            'auto_topic_relay = multi_robot_relay.auto_topic_relay:main',
        ],
    },
)
