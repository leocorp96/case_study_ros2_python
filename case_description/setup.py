import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'case_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Correctly include all files in urdf/meshes and subdirectories
        *[
            (os.path.join('share', package_name, os.path.dirname(f)), [f])
            for f in glob('urdf/meshes/**/*', recursive=True) if os.path.isfile(f)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ephson Guakro',
    maintainer_email='leocorp96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
