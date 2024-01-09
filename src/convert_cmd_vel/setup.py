from setuptools import setup

package_name = 'convert_cmd_vel'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='M Viswanathan',
    author_email='viswanathan.m@gmail.com',
    maintainer='M Viswanathan',
    maintainer_email='viswanathan.m@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='/cmd_vel to /diff_cont/cmd_vel',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert_cmd_vel = convert_cmd_vel.cmd_to_diff_cont:main'
        ],
    },
)