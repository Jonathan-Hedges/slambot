from setuptools import setup

package_name = 'slambot'

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
    maintainer='xpctd',
    maintainer_email='xpctd@todo.todo',
    description='Roomba Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'avoidance = slambot.avoidance:main',
        	'laser = slambot.read_laser:main',
        	'launch = slambot.gazebo.launch:main',
        ],
    },
)
