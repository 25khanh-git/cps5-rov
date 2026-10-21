from setuptools import setup

package_name = 'rov_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nguyen Duy Khanh',
    maintainer_email='khanh23022007@gmail.com',
    description='Điều khiển tay cho ArduSub qua MAVROS RC override.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = rov_teleop.teleop_node:main',
            'keyboard_teleop = rov_teleop.keyboard_node:main',
        ],
    },
)
