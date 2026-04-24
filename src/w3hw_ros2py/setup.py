from setuptools import find_packages, setup
from glob import glob

package_name = 'w3hw_ros2py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher_node = w3hw_ros2py.publisher_node:main',
            'subscriber_node = w3hw_ros2py.subscriber_node:main',
            'server = w3hw_ros2py.service_server:main',
            'client = w3hw_ros2py.service_client:main',
        ],
    },
)

