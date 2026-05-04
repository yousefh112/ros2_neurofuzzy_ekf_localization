import os
from setuptools import setup, find_packages

package_name = 'robot_description_pkg'

def generate_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Define every root folder that needs to be copied to the install space
    directories_to_install = ['launch', 'worlds', 'urdf', 'models', 'meshes']
    
    for directory in directories_to_install:
        for (path, _, filenames) in os.walk(directory):
            files = [os.path.join(path, f) for f in filenames]
            if files:
                target_path = os.path.join('share', package_name, path)
                data_files.append((target_path, files))
                
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yousefabdelhady',
    maintainer_email='yousef_hesham.yh112@icloud.com',
    description='ROS 2 Sensor Fusion, EKF, and ML Localization',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c1_node = robot_description_pkg.controllers.c1_node:main',
            'c2_node = robot_description_pkg.controllers.c2_node:main',
        ],
    },
)