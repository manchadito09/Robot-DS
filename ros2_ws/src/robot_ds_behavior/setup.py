from setuptools import find_packages, setup

package_name = 'robot_ds_behavior'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Manchado',
    maintainer_email='manchadito09@users.noreply.github.com',
    description='Robot-DS guide behavior: request -> autonomous Nav2 navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run robot_ds_behavior <name>
            'guide = robot_ds_behavior.guide:main',
            'brain = robot_ds_behavior.brain:main',
            'talk = robot_ds_behavior.talk:main',
            'explore = robot_ds_behavior.explore:main',
        ],
    },
)
