from setuptools import find_packages, setup

package_name = 'waypoint_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoint_manager.yaml']),
        ('share/' + package_name + '/config', ['config/waypoint_saver.yaml']),
        ('share/' + package_name + '/config', ['config/waypoint_visualizer.yaml']),  
        ('share/' + package_name + '/config', ['config/waypoint_skipper.yaml']),      
        ('share/' + package_name + '/launch', ['launch/waypoint_manager.launch.py']),
        ('share/' + package_name + '/launch', ['launch/waypoint_saver.launch.py']),
        ('share/' + package_name + '/launch', ['launch/waypoint_visualizer.launch.py']),
        ('share/' + package_name + '/launch', ['launch/waypoint_skipper.launch.py']),      
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuma',
    maintainer_email='kazuma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager = waypoint_manager.waypoint_manager:main',
            'waypoint_saver = waypoint_manager.waypoint_saver:main',
            'waypoint_visualizer = waypoint_manager.waypoint_visualizer:main',
            'waypoint_skipper = waypoint_manager.waypoint_skipper:main'
        ],
    },
)
