from setuptools import setup, find_packages

PACKAGE_NAME = 'stp'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oswinso',
    maintainer_email='oswinso@gmail.com',
    description='Rewrite of the gameplay library.',
    entry_points={
        'console_scripts': [],
    },
)
