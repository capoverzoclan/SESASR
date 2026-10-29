from setuptools import find_packages, setup

package_name = 'lab01_pkg'

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
    maintainer='verzo',
    maintainer_email='verzo@todo.todo',
    description='SESASR lab01',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = lab01_pkg.controller_member_function:main',
            'localization = lab01_pkg.localization_member_function:main',
            'reset_node = lab01_pkg.reset_node_member_function:main',
            'controller_reset = lab01_pkg.controller_reset_member_function:main',
            'localization_reset = lab01_pkg.localization_reset_member_function:main',
        ],
    },
)
