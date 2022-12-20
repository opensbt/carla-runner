FROM ros:melodic

WORKDIR /opt/workspace

RUN mkdir src && mkdir share

RUN apt update
RUN apt install -y \
    software-properties-common \
    ca-certificates \
    gpg \
    wget
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor - | \
    sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
RUN apt update
RUN apt install -y cmake libjsoncpp-dev python3.8 python3-pip
RUN apt-get update
RUN apt install -y python3-catkin-pkg-modules python3-rospkg-modules
RUN python3.8 -m pip install --upgrade pip
ADD . /opt/ff1_carla
RUN pip install -r /opt/ff1_carla/requirements.txt

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc