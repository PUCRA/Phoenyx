from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final'

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
    maintainer='naide',
    maintainer_email='naide@estudiantat.upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localizacion_final = final.localizacion_final:main',
            'brain_final = final.brain_final:main'
        ],
    },
)