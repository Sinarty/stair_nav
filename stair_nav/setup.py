from glob import glob
from setuptools import find_packages, setup

setup(
    name='stair_nav', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stair_nav']),
        ('share/stair_nav', ['package.xml']),
        ('share/stair_nav/config', glob('config/*.yaml')),
        ('share/stair_nav/launch', glob('launch/*.launch.py')),
        ('share/stair_nav/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Stair navigation team', maintainer_email='maintainer@example.com',
    description='Segment navigation and a synthetic, hardware-free ROS 2 demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'segment_navigation = stair_nav.segment_node:main',
        'task_manager = stair_nav.task_node:main',
        'sim_executor = stair_nav.sim_node:main',
    ]},
)
