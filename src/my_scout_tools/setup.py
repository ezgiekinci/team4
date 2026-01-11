from setuptools import setup

package_name = 'my_scout_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch dosyası ekleyeceksen bu satırı açık bırak; yoksa sorun değil.
        # ('share/' + package_name + '/launch', ['launch/cmd_vel.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diakamos',
    maintainer_email='',
    description='CmdVel publisher for SCOUT 2.0',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_node = my_scout_tools.cmd_vel_node:main',
        ],
    },
)

