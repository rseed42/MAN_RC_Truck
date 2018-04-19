#!/bin/bash

IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
echo "Using IP: $IP"

# Register this IP address
xhost + $IP

# Run the app
docker run \
  --name truck-app \
  --rm \
  -v $(pwd):/home truck-app:0.1 \
  -e DISPLAY=$IP:0
