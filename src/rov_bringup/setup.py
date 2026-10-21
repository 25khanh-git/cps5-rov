from glob import glob

from setuptools import setup

package_name = 'rov_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/ardusub', glob('config/ardusub/*.parm')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nguyen Duy Khanh',
    maintainer_email='khanh23022007@gmail.com',
    description='Launch, cấu hình và kiểm tra simulation cho CPS 5.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_sim = rov_bringup.check_sim:main',
            'chase_camera = rov_bringup.chase_camera:main',
        ],
    },
)
