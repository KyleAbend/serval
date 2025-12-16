from setuptools import setup, find_packages

package_name = 'serval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/serval.launch.py'
        ]),
        ('share/' + package_name + '/description', [
            'description/serval.urdf.xacro'
        ]),
        ('share/' + package_name + '/config', [
            'config/serval_control.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='kyle-abend',
    author_email='kyle.abend.robots@gmail.com',
    maintainer='kyle-abend',
    maintainer_email='kyle.abend.robots@gmail.com',
    description='Serval robot control package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_teleop = joint_teleop:main',
        ],
    },
)
