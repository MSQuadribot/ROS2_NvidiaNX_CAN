from setuptools import setup

package_name = 'canbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qb',
    maintainer_email='maximilien.sonnic@quadribot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reader = canbot.can_reader:main',
            'subscriber = canbot.can_listener:main',
            'keyboard = canbot.can_input:main',
            'output = canbot.can_output:main',
            'controller = canbot.can_controller:main',
            'variables = canbot.can_variables:main',
            'joystick = canbot.can_joystick:main',
        ],
    },
)
