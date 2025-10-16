from setuptools import find_packages, setup

package_name = 'simple_pyrobosim_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Devis Dal Moro',
    maintainer_email='devis.dalmoro@eurecat.org',
    description='A wrapper on top of pyrobosim ros for ROSCON ES25',
    license='TODO: License declaration',
    extras_require={
        # 'test': [
        #     'pytest',
        # ],
    },
    entry_points={
        'console_scripts': [
            'simple_pyrobosim_ros = simple_pyrobosim_ros.simple_pyrobosim_wrapper:main',
        ],
    },
)
