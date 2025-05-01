from setuptools import find_packages, setup

package_name = 'audio_testing_wav'

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
    maintainer='vedant',
    maintainer_email='vedantchoudhary16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wav_to_uint8_node = audio_testing_wav.audio_sending_node:main',
            'test_node = audio_testing_wav.audio_testing_node:main',
        ],
    },
)
