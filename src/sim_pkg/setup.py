from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sim_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register package with ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Include package.xml
        ('share/' + package_name, ['package.xml']),

        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('sim_pkg/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jliu',
    maintainer_email='jliu6162@gmail.com',
    description='ROS 2 + Isaac Sim demo package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'box_cmd_node = sim_pkg.scripts.box_cmd_node:main',
            'run_isaacsim = sim_pkg.scripts.run_isaacsim:main',
            'spawn_box = sim_pkg.scripts.spawn_box:main',
            'open_isaacsim_stage = sim_pkg.scripts.open_isaacsim_stage:main',
            'stabilizer_node = sim_pkg.scripts.stabilizer_pid:main',
            'cart_effort_node = sim_pkg.scripts.cart_effort_node:main',
            'cart_policy_node = sim_pkg.scripts.cart_policy:main',
            'cart_debug_node = sim_pkg.scripts.cart_debug_sub:main',
            'stabilizer_policy = sim_pkg.scripts.stabilizer_policy:main'
        ],
    },
)
