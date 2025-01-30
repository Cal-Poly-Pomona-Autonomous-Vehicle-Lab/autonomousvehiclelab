from setuptools import setup

package_name = 'web_control_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'flask>=2.0.0',
        'flask-socketio>=5.0.0',
        'python-socketio>=5.0.0',
        'eventlet>=0.33.0',
        'rclpy>=0.10.0',
        'ros-geometry-msgs>=0.12.0',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Abhishek Vishwakarma',
    maintainer_email='avishwakarma@cpp.edu',
    description='Web control server node for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = web_control_server.server:main',
        ],
    },
)
