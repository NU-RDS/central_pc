from setuptools import setup

package_name = 'simple_vr_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'openvr'],
    zip_safe=True,
    maintainer='NU-Haptics-local',
    maintainer_email='kevin.nella@northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'VR_publisher = simple_vr_driver.VR_publisher:main',
        ],
    },
)
