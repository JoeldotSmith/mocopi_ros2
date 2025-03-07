from setuptools import setup

package_name = 'mocopi_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
