from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'aruco_patroller_task'


model_data = []
for root, dirs, files in os.walk('models'):
    if not files:
        continue  
    src_files = [os.path.join(root, f) for f in files]
    install_dir = os.path.join('share', package_name, root)
    model_data.append((install_dir, src_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ] + model_data,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='valentin.simundic@ferit.hr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_localization = aruco_patroller_task.aruco_localization_node:main',
            'waypoint_navigator = aruco_patroller_task.waypoint_navigator:main',
            'aruco_pose_detector = aruco_patroller_task.aruco_pose_detector:main',
            'aruco_patroller = aruco_patroller_task.aruco_patroller:main',
        ],

    },
)
