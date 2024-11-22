from setuptools import find_packages, setup

package_name = 'map_pkg'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico Saccon & Placido Falqueto',
    maintainer_email='{enrico.saccon,placido.falqueto}@unitn.it',
    description='Package to publish details regarding the map',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_map_pgm = scripts.create_map_pgm:main',
            'generate_config_file = scripts.generate_config_file:main'
        ],
    },
)
