from setuptools import setup
from glob import glob

package_name = 'minitrone_plant'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/xml', glob('xml/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryung',
    maintainer_email='jjungsoo2022@gmail.com',
    description='MuJoCo plant loader',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minitrone_plant = minitrone_plant.plant:main',
            'minitrone_palm_teleop = minitrone_plant.palm_teleop:main',
            'minitrone_external_wrench_plot = minitrone_plant.minitrone_external_wrench_plot:external_wrench_main',
            'minitrone_topic_plot = minitrone_plant.minitrone_external_wrench_plot:main',
        ],
    },
)
