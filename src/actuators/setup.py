from setuptools import find_packages, setup

package_name = 'actuators'

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
    maintainer='szogabaha',
    maintainer_email='szolnokgabcsi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mycobot_driver = actuators.__init__:start_driver",
            "mock_mycobot_driver = actuators.__init__:start_mock_driver",
            "hil_driver = actuators.__init__:start_hil_driver",
        ],
    },
)
