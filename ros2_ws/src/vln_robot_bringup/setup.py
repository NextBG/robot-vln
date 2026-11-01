from glob import glob

from setuptools import setup

package_name = 'vln_robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['wide_cam_node = vln_robot_bringup.wide_cam_node:main']},
    zip_safe=True,
    maintainer='NextBG',
    maintainer_email='nextb@example.com',
    description='Bringup for the RoboMaster S1 VLN robot',
    license='MIT',
)
