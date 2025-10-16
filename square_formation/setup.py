from setuptools import setup

package_name = 'square_formation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/formation_setup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pcandelas98@gmail.com',
    description='Square formation action and coordination package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator = square_formation.coordinator:main',
            'move_to_server = square_formation.move_to_server:main',
            'move_to_client = square_formation.move_to_client:main',
            'targets = square_formation.targets:main',
        ],
    },
)