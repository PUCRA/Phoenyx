from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'guiado'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de configuración YAML
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Incluir archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santi',
    maintainer_email='santiago.pallares@estudiantat.upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localizacion_aruco = guiado.localizacion_aruco:main',
            'brain_guiado = guiado.brain_guiado:main',
            'test_localizacion = guiado.test_localizacion:main'
        ],
    },
)