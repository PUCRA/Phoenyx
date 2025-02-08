from setuptools import setup
from glob import glob
import os
package_name = 'phoenyx_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))), 
        ('share/' + package_name + '/conf', glob(os.path.join('conf', '*.yaml'))), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pucra',
    maintainer_email='german.bueno@estudiantat.upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_pub = phoenyx_nodes.imu_pub:main',
            'killer_node = phoenyx_nodes.KillerNode:main',
            'Vel_pub = phoenyx_nodes.Vel_pub:main',
            'camera_node = phoenyx_nodes.camera:main',
        ],
    },
)
