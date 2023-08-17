from setuptools import setup

package_name = 'py_pubsubpub_3node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
              'node1pub = py_pubsubpub_3node.node1_pub:main',
        	'node2subpub = py_pubsubpub_3node.node2_sub_pub:main',
        	'node3sub = py_pubsubpub_3node.node3_sub:main',
        ],
    },
)
