from setuptools import find_packages, setup

package_name = 'gesture_control_pkg'

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
    maintainer='pk2k5',
    maintainer_email='pk2k5@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_node = gesture_control_pkg.gesture_controller:main',
            'udp_bridge = gesture_control_pkg.udp_bridge:main',
            'movements_node = gesture_control_pkg.movements:main',
            'remote_node = gesture_control_pkg.remote:main'
        ],
    },
)
