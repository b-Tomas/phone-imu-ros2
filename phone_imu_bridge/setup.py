from setuptools import find_packages, setup

package_name = 'phone_imu_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomás Badenes',
    maintainer_email='tomasbadenes@gmail.com',
    description='Bridge for streaming phone IMU data to ROS2 via WebSockets',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_server_node = phone_imu_bridge.imu_server_node:main',
        ],
    },
)
