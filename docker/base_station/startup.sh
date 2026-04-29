#!/usr/bin/bash

# Start the pigpio daemon to allow GPIO access
sudo pigpiod

# Wait for ethernet connection
while ! ping -c 1 -W 1 10.42.0.1; do
    echo "Waiting for network connection..."
    sleep 1
done

# Start the docker container for the base station
docker run -d \
    --name base-station \
    --restart unless-stopped \
    --network host \
    --privileged \
    rj_base_station:latest \
    base_station.launch.py