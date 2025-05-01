from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rolle_langchain_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all other directories needed for the package
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedant',
    maintainer_email='vedantchoudhary16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'navigation_node = rolle_langchain_agent.rolle_agent:main',
            # 'command_processor = rolle_langchain_agent.command_processor:main',
            # 'rolle_integration = rolle_langchain_agent.rolle_integration:main',
            'single_shot = rolle_langchain_agent.single_shot_agent:main',
            'test_single_shot = rolle_langchain_agent.single_shot_test:main',
        ],
    },
)
