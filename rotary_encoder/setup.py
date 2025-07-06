from setuptools import find_packages, setup

package_name = 'rotary_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeremy',
    maintainer_email='jeremy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "angle_publisher = rotary_encoder.angle_publisher:main",
            "angle_plotter = rotary_encoder.angle_plotter:main",
            "angle_publisher_serial = rotary_encoder.angle_publisher_serial:main",
            "angle_logger_serial = rotary_encoder.angle_logger_serial:main",
            "angle_plotter_serial = rotary_encoder.angle_plotter_serial:main",
        ],
    },
)
