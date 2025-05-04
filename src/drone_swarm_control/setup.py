from setuptools import find_packages, setup

package_name = 'drone_swarm_control'

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
    maintainer='mirza',
    maintainer_email='y.mirza.altay@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'core_drone_controller_node = drone_swarm_control.nodes.core_drone_controller_node:main',
        ],
    },
)
