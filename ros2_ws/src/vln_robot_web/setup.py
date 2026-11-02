import os
from glob import glob

from setuptools import setup

package_name = 'vln_robot_web'


def tree(src, dst):
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join(dst, os.path.relpath(root, src)).rstrip('/.'),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + tree('static', 'share/' + package_name + '/static'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NextBG',
    maintainer_email='nextb@example.com',
    description='Web dashboard for monitoring and remote control of the robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_node = vln_robot_web.web_node:main',
        ],
    },
)
