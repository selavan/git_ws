from setuptools import find_packages, setup

package_name = 'hwt905'

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
    maintainer='miccy',
    maintainer_email='miccy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'hwt905_node    = hwt905.wit_normal_ros:main',
        'hwt905_node_v1 = hwt905.wit_normal_ros_v1:main',
        ],
    },
)
