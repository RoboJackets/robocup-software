# Robocup Dev Containers

## Description

To make developing with the RoboJackets robocup software eaiser, a development image has been created to encapsulate and standardize the development of software.  This docker image mounts the local robocup-software code to /ws and comes complete with most of the league software we use.

## Running

### Linux

For linux, you don't need to install any dependencies and you should be able to start up the docker development container by running:

```bash
./docker/docker-build.sh && ./docker/docker-run-linux.sh
```

### MacOs

For MacOs, you will need to install an X server (for example [XQuartz](https://www.xquartz.org/)) and then you can start up the docker development container by running:

```bash
./docker/docker-build.sh && ./docker/docker-run-mac.sh
```

### Windows
For Windows, you will also need to install an X server.  You should also be using WSL with Docker Desktop.  Then you can start up the docker development container by running:

```bash
.\docker\docker-build.sh && .\docker\docker-run-windows.sh
```

## Developing

Once the development container is crunning, you can connect to the container via vscode (or your preferred ide) and begin developing as normal.
