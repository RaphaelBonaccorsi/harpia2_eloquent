from setuptools import setup

package_name = 'mission_goal_manager'

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
    maintainer='raphael',
    maintainer_email='raphael@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_goal_manager_server_temporal_planing = mission_goal_manager.mission_goal_manager_server_temporal_planing:main',
            'mission_goal_manager = mission_goal_manager.mission_goal_manager_server:main'
        ],
    },
)
