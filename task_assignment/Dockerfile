FROM ubuntu:22.04

# Set the working directory
WORKDIR /assignment

RUN apt-get update \
    && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update \
    && apt-get install -y \
    sudo \
    build-essential \
    cmake \
    git \
    python3 \
    python3-pip \
    libgl1 libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && pip3 install \
        numpy \
        pybullet \
        pin \
        matplotlib 

# Clone the Git repository into the working directory
RUN git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git

# Default command (can be changed as needed)
CMD ["/bin/bash"]