from setuptools import setup

package_name = 'mapping_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/mapping_controller_launch.py']),
        ('share/' + package_name + '/resource',
            ['resource/random_bounce.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Debby Nirwan',
    author_email='debby_nirwan@yahoo.com',
    maintainer='Debby Nirwan',
    maintainer_email='debby_nirwan@yahoo.com',
    keywords=['ROS2', 'Webots', 'Probability Mapper',
              'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Mapping Controller for epuck on Webots Simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_bounce = mapping_controller.random_bounce:main',
            'mission_controller = mapping_controller.mission_controller:main',
            'probability_mapper = mapping_controller.probability_mapper:main',
        ],
    },
)
