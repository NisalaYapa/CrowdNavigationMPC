from setuptools import find_packages, setup

package_name = 'smrr_crowdnav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['smrr_crowdnav', 'smrr_crowdnav.include']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nisala',
    maintainer_email='nisala@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node_new = smrr_crowdnav.control_node:main',  
            'test_publisher = smrr_crowdnav.test_publisher:main',
            'test_command = smrr_crowdnav.testcommands:main'
        ],
    },
)
