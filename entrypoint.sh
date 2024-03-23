#!/bin/bash

# Update packages
apt-get update && apt-get upgrade -y

echo "Hello World!"
source /colcon_ws/install/setup.bash

exec "$@"