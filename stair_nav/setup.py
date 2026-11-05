from glob import glob
from setuptools import find_packages, setup

setup(
    name='stair_nav', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stair_nav']),
        ('share/stair_nav', ['package.xml']),
        ('share/stair_nav/demo/config', glob('stair_nav/demo/config/*.yaml')),
        ('share/stair_nav/demo/config/maps', glob('stair_nav/demo/config/maps/*.yaml')),
        ('share/stair_nav/demo/rviz', glob('stair_nav/demo/rviz/*.rviz')),
        ('share/stair_nav/config', glob('config/*.yaml')),
        ('share/stair_nav/launch', glob('launch/*.launch.py') + glob('stair_nav/demo/launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    python_requires='>=3.8',
    zip_safe=True,
    maintainer='Stair navigation team', maintainer_email='maintainer@example.com',
    description='Segment navigation and a synthetic, hardware-free ROS 2 demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'segment_navigation = stair_nav.segment_node:main',
        'task_manager = stair_nav.task_node:main',
        'mission_client = stair_nav.mission_client:main',
        'sim_executor = stair_nav.demo.sim_node:main',
        'obstacle_monitor = stair_nav.obstacle_monitor_node:main',
        'demo_obstacles = stair_nav.demo.obstacles_node:main',
        'm20_localization = stair_nav.m20_localization_node:main',
        'm20_motion_adapter = stair_nav.m20_motion_node:main',
    ]},
)
