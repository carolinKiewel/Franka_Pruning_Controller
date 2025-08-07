from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

# Helper to recursively gather all launch files preserving structure
def get_launch_files():
    launch_files = []
    for root, dirs, files in os.walk('robot_launch'):
        for file in files:
            if file.endswith('.launch.py'):
                filepath = os.path.join(root, file)
                # Install path relative to share/robot_bringup/
                install_path = os.path.join('share', package_name, filepath)
                launch_files.append((install_path, [filepath]))
    return launch_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/amment_index/resource_index/packages", ["resource/" + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        # Add this line to include all nested launch files
        #('share/' + package_name + '/robot_launch/robot_moveit', 
            #glob('robot_launch/robot_moveit/*.launch.py')),
        (os.path.join("share", package_name, "config"), glob("launch/*launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='missikiewel@gmail.com',
    description='Robot bringup launch stack for modular FR3 setup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
