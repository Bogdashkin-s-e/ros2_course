from setuptools import setup

package_name = 'gazebo_controller_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Wall follower for TurtleBot3 in Gazebo',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = gazebo_controller_pkg.wall_follower:main',
        ],
    },
)