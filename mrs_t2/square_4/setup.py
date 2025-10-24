from setuptools import find_packages, setup

package_name = 'square_4'

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
    maintainer='pablo',
    maintainer_email='pcandelas98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_motion = square4.simple_motion:main',
            'simple_square_v1 = square4.simple_square_v1:main',
            'coordinated_square_v1 = square4.coordinated_square_v1:main',
        ],
    },
)
