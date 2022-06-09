from setuptools import setup

package_name = 'openspot_teleop_kb'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    py_modules=[
        'openspot_teleop_kb'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transforms3d'],
    zip_safe=True,
    maintainer='yandy',
    maintainer_email='yandy.ding@gmail.com',
    description='openspot teleoperation by keyboard',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = openspot_teleop_kb:main'
        ],
    },
)
