from setuptools import setup

package_name = 'so_arm100_control'

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
    entry_points={
        'console_scripts': [
            'pick_and_place = so_arm100_control.pick_and_place:main',
            
            'move_arm = so_arm100_control.move_arm:main',
        ],
    },
)
