FROM ros:galactic-ros-core-focal
LABEL maintainer="lzhang3@caltech.edu"

# update & install Basilisk pre-reqs
RUN apt-get update && apt-get install -y git && \
    apt-get install python3-tk -y && \
    apt-get install python3-pip -y && \
    apt-get install python3.8-venv -y && \
    apt-get install swig -y && \
    apt-get install sudo -y && \
    apt-get install python3-colcon-common-extensions -y && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-galactic-rviz2

ARG UID
# https://stackoverflow.com/a/49848507
RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u $UID ubuntu
RUN echo 'ubuntu:p' | chpasswd
USER ubuntu
WORKDIR /home/ubuntu

# source and add ROS2 in bashrc
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# install Basilisk
RUN git clone https://github.com/AVSLab/basilisk.git && \
    cd basilisk && python3 -m venv .venv

SHELL ["/bin/bash", "-c"]
# install python deps
RUN source basilisk/.venv/bin/activate && pip3 install wheel && \
                                          pip3 install cmake conan==1.54 && \
                                          pip3 install pandas matplotlib parse

# RUN source basilisk/.venv/bin/activate && pip3 conan

# should take bt 15-20 mins
RUN cd basilisk && source .venv/bin/activate && python3 conanfile.py --opNav False --vizInterface True --buildProject True

# run again? not sure why I need to do this, but it gets the vizualizer to actually work.
# should take around 1ish mins.
# RUN cd basilisk && source .venv/bin/activate && python3 conanfile.py --opNav False --vizInterface True --buildProject True

# One more time maybe?
# RUN cd basilisk && source .venv/bin/activate && python3 conanfile.py --opNav False --vizInterface True --buildProject True

# TODO: install instant-ngp

CMD ["bash", "-i"]