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
RUN apt install -y cmake libjsoncpp-dev

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
