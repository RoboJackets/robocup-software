# RoboCup Reinforcement Learning

This is the start of RL at RoboCup!

Purely for my own edification—because I have never used it before—I'll be conducting this work in python with MuJoCo.

## Environment Setup

Because our stack requires a specific system python, I won't use a new python just a venv on our normal python. To be clear, you should use any source of python that resolves to `3.10.x` and has the venv package (`sudo apt get python3.10-venv`).

```
cd ./rl_environment
python3 -m venv .venv
  ^ or whatever keyword resolves to your python (py, py3, py310, python, etc.)
source .venv/bin/activate
```

This tampers with your python source, so be careful with the rest of the stack while this is active. (`deactivate` to deactivate)

Then install the packages.

```
pip install -r requirements.txt
```
