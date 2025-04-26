from setuptools import find_packages, setup
import os
import glob

package_name = 'phoenyx_nodes'

# Usar glob para encontrar todos los archivos .yaml en config
config_files = glob.glob(os.path.join('config', '*.yaml'))

# Usar glob para encontrar todos los archivos .py en launch
launch_files = glob.glob(os.path.join('launch', '*.py'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir todos los archivos de configuración (YAML)
        ('share/' + package_name + '/config', config_files),
        # Incluir todos los archivos de lanzamiento (launch)
        ('share/' + package_name + '/launch', launch_files),
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
            'front_distance = phoenyx_nodes.front_distance:main',
            'middle_error = phoenyx_nodes.middle_error:main',
            'avanzar_pasillo = phoenyx_nodes.avanzar_pasillo:main',
            'guiado = phoenyx_nodes.guiado:main',
            'leds = phoenyx_nodes.leds:main'
        ],
    },
)
