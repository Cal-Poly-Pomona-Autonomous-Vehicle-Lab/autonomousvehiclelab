from setuptools import find_packages, setup

package_name = 'generic_topic_tools'

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
    maintainer='Abhishek Vishwakarma',
    maintainer_email='abhishekvishwakarma218@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generic_relay_node = generic_topic_tools.generic_relay_node:main'
        ],
    },
)
