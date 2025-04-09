from setuptools import find_packages, setup
import glob
import os
package_name = 'planificador'
package_name = 'planificador'
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
        # #Incluir todos los archivos de lanzamiento (launch)
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icehot03',
    maintainer_email='pol.p.c@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planificador = planificador.planificador:main',
        ],
    },
)
