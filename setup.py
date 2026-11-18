from setuptools import setup
from glob import glob

package_name = 'k9_cliff_pkg'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='K9 project',
    maintainer_email='maintainer@example.invalid',
    description='Six-channel VL53L0X K9 cliff sensor monitor',
    license='Apache-2.0',
    entry_points={'console_scripts': ['cliff_detector = k9_cliff_pkg.cliff_detector:main']},
)
