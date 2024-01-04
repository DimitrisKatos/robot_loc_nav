from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_loc_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),
        (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.xacro'))),
         (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.gazebo'))),
        (os.path.join('share',package_name,'worlds'),
         glob(os.path.join('worlds','*.world'))),
        (os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.dae'))),
        (os.path.join('share',package_name,'config'),
         glob(os.path.join('config','*.yaml'))),
        (os.path.join('share',package_name,'maps'),
         glob(os.path.join('maps','*.yaml'))),
        (os.path.join('share',package_name,'maps'),
         glob(os.path.join('maps','*.pgm'))),
        (os.path.join('share',package_name,'rviz'),
         glob(os.path.join('rviz','*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='katos',
    maintainer_email='katos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive = robot_loc_nav.diff_drive:main'
        ],
    },
)
