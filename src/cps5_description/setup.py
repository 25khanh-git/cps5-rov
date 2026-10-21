from glob import glob

from setuptools import setup

package_name = 'cps5_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nguyen Duy Khanh',
    maintainer_email='khanh23022007@gmail.com',
    description='Thông số vật lý và bộ sinh SDF cho robot CPS 5.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_summary = cps5_description.model:main',
        ],
    },
)
