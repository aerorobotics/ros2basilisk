#!/bin/sh
# TODO: Update the name of the docker image, keeping it like this for testing purposes.
docker build --no-cache --pull --build-arg UID=`id -u` --network=host -t kmatsuka/ros2basilisk - < Dockerfile
