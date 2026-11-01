from glob import glob

from setuptools import setup

package_name = 'vln_robot_client'

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
    zip_safe=True,
    maintainer='NextBG',
    maintainer_email='nextb@example.com',
    description='Robot-side VLN client',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vln_client = vln_robot_client.vln_client_node:main',
            'lidar_depth_projector = vln_robot_client.lidar_depth_projector:main',
            'safety_stop = vln_robot_client.safety_stop:main',
        ],
    },
)
