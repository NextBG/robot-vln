from glob import glob

from setuptools import setup

package_name = 'vln_robot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/routes', glob('routes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NextBG',
    maintainer_email='nextb@example.com',
    description='GPS route following (AMap-planned routes) for the RoboMaster S1 robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_waypoint_follower = vln_robot_nav.gps_waypoint_follower:main',
            'nav_sim = vln_robot_nav.nav_sim:main',
        ],
    },
)
