#!/bin/bash

# Update packages
apt-get update && apt-get upgrade -y --no-install-recommends 

echo "Hello World!"
source /colcon_ws/install/setup.bash

exec "$@"