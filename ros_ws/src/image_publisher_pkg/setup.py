from setuptools import setup

package_name = 'image_publisher_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'image_transport'],
    entry_points={
        'console_scripts': [
            'image_publisher_node = image_publisher_pkg.image_publisher_node:main',
        ],
    },
)
