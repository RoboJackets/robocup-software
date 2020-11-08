from setuptools import find_packages, setup

PACKAGE_NAME = "rj_gameplay"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oswinso",
    maintainer_email="oswinso@gmail.com",
    description="Rewrite of the gameplay library.",
    entry_points={
        "console_scripts": [],
    },
)
