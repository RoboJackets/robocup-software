"""Setup script for rj_rl reinforcement learning package."""
from setuptools import setup, find_packages

setup(
    name="rj_rl",
    version="0.1.0",
    description="Reinforcement learning system for RoboCup SSL strategy",
    packages=find_packages(),
    package_data={"rj_rl": ["proto_gen/*.py"]},
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "protobuf>=3.20.0",
    ],
    extras_require={
        "test": ["pytest>=6.0"],
    },
    entry_points={
        "console_scripts": [
            "rj-rl-train=scripts.train:main",
        ],
    },
)
