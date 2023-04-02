# Docker Instruction

Instead of following the manual installation, you can use Docker container to test. The image has main dependencies such as ROS2 Galactic and Basilisk pre-installed. You need to create a ROS2 workspace containing ros2basilisk locally on your computer. When running a container, you will map this local workspace (e.g. `/home/UserName/basilisk_ws`) to the workspace directory within the container `/home/ubuntu/example_ws`.

## Setup ROS2 Worskpace

1. If ROS2 workspace does not already exists, create one `mkdir -p basilisk_ws/src`
1. Clone this repo into workspace `cd basilisk_ws/src && git clone https://github.com/aerorobotics/ros2basilisk`

## Get an image from Dockerfile
Skip this step if you are using our [Docker Image](https://hub.docker.com/repository/docker/kmatsuka/ros2basilisk/general). Alternatively, you can build your image from Dockerfile provided.
1. Change UserName/ImageName in `docker_build.sh`
1. `cd ros2basilisk/docker` (so now your path (from `pwd`) should be: "/home/user/example_ws/src/ros2basilisk/docker/", assuming user is your user and you followed the above commands from your home folder.)
1. `./docker_build.sh` to create a Docker Image with ROS2 Galactic environment and Basilisk. Takes long time (~30min) but you need to do this once.

## Create and run a new docker container 
Repeat these steps for each time a new container is created.
1. If you built your own Docker image, change UserName/ImageName in `docker_run.sh`
1. Go to the root of ROS2 worspace directory (`cd ../../../ `). Your directory should be something like `/home/UserName/basilisk_ws`. This step is important for mapping the correct local directory to the directory within the container.
1. Run a new docker container (`bash src/ros2basilisk/docker/docker_run.sh`). This enters ubuntu 20.04 environment and maps the local ROS2 workspace to the container. 
1. From inside the container, activate the Python Virtual Environment and install some Python dependencies (`source ~/basilisk/.venv/bin/activate && pip3 install -r ~/example_ws/src/ros2basilisk/requirements.txt`).

## Running demos inside the container.
Repeat these steps for each time you run the demo (inside the same docker container).
1. If you have not done so, build ROS2 workspace (`cd example_ws/ && colcon build`).
1. Ensure virtual environment is activated and source ROS2 environment (`source ~/basilisk/.venv/bin/activate && source ~/example_ws/install/local_setup.bash`).
1. Run the demo (`ros2 launch ros2basilisk demo_launch.py ws:="/home/ubuntu/example_ws/"`).
