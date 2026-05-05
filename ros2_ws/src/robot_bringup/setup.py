from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- AGGIUNGI QUESTE DUE RIGHE ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # --------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pietro',
    maintainer_email='s353110@studenti.polito.it',
    description='Package per la fusione GPS-INS-Odom con Complementary Filter',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # Assicurati che il nome a sinistra sia quello usato nel Launch file
            'complementary_filter = robot_bringup.complementary_filter:main',
            'mock_ekf = robot_bringup.mock_ekf:main',
        ],
    },
)