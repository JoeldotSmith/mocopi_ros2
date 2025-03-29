from setuptools import setup
from glob import glob

package_name = 'mocopi_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/config', ['config/mocopi_view.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/g1_29dof_lock_waist.urdf']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf', ['urdf/g1_29dof_lock_waist.xml']),
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*.STL')),
        ('share/' + package_name + '/urdf/images', glob('urdf/images/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joelsmith',
    maintainer_email='23338559@student.uwa.edu.au',
    description='Connect mocopi motion tracking to Ros2',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocopi_receiver = mocopi_ros2.mocopi_receiver:main',
            'log_mocopi_receiver = mocopi_ros2.log_version:main',
        ],
    },
)

