from setuptools import find_packages
from setuptools import setup

package_name = 'control_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/launch', ['launch/trajectory.launch.py']),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Control signal nodes c1 and c2 for mobile robot trajectory generation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_controller = control_pkg.trajectory_controller:main',
        ],
    },
)