from setuptools import setup

package_name = 'mission_planning'

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
            'mission_planner_server = mission_planning.mission_planner_server:main',
            'test_add_goal = mission_planning.test_add_goal:main',
            'test_client (copy) = mission_planning.test_client (copy):main',
            'test_client = mission_planning.test_client:main',
        ],
    },
)
