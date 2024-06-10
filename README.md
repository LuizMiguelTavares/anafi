
# Setting up the Environment

## Step 1: Create a Custom Docker Image

First, we need to create a custom Docker image based on Ubuntu 20.04 with a user named `anafi`. This image will have all necessary dependencies pre-installed.

### Dockerfile

Create a file named `Dockerfile` with the following content:

```dockerfile
FROM ubuntu:focal

# Set the environment variable to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary packages
RUN apt-get update && \
    apt-get install -y sudo git python3 python3-dev python3-pip python3-setuptools python3-wheel build-essential cmake libavcodec-dev libavformat-dev libswscale-dev libavdevice-dev

# Create a new user 'anafi' with sudo privileges
RUN useradd -m -s /bin/bash anafi && \
    echo 'anafi ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the new user
USER anafi

# Set the default command to bash
CMD ["/bin/bash"]
```

## Step 2: Build the Docker Image

Open a terminal in the same directory as the `Dockerfile` and run the following command to build the Docker image:

```sh
docker build -t ubuntu_anafi:latest .
```

## Step 3: Create a Script to Run the Docker Container

Create a script named `run_anafi_container.sh` with the following content. This script will run the Docker container with appropriate settings.

```sh
#!/bin/bash

# Uncomment the --gpus all line if you have a GPU-enabled Docker setup
docker run -it \
  --gpus all \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/machine-id:/etc/machine-id \
  -v /var/run/dbus:/var/run/dbus \
  -v /dev/dri:/dev/dri \
  -v /dev/snd:/dev/snd \
  --device /dev/dri \
  --device /dev/snd \
  --name anafi_container \
  ubuntu_anafi
```

## Step 4: Make the Script Executable and Run It

Open a terminal in the same directory as the `run_anafi_container.sh` script and run the following commands to make it executable and run the container:

```sh
chmod +x run_anafi_container.sh
./run_anafi_container.sh
```

## Step 5: Update and Install Dependencies Inside the Container

Once inside the container, update the package list and install necessary dependencies:

```sh
sudo apt update
sudo apt install software-properties-common curl
```

## Step 6: Install Python 3.9

Install Python 3.9 and set it as the default Python 3 version:

```sh
sudo apt install python3.9 python3.9-distutils python3.9-venv
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 2
sudo update-alternatives --config python3 # Select Python 3.9
```

## Step 7: Install pip for Python 3.9

Download and install pip for Python 3.9:

```sh
sudo curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python3.9 get-pip.py
```

## Step 8: Install Olympe

Install the Olympe package, which is required for working with the Parrot drone:

```sh
pip3 install https://github.com/Parrot-Developers/olympe/releases/download/v7.5.0/parrot_olympe-7.5.0-py3-none-manylinux_2_27_x86_64.whl
echo "export PATH=\"$HOME/.local/bin:$PATH\"" >> ~/.bashrc
source ~/.bashrc
```

## Step 9: Install ROS Noetic

Add the ROS package repository, update the package list, and install ROS Noetic:

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 10: Fix the netifaces Issue

Reinstall the `netifaces` package to avoid potential issues:

```sh
sudo pip uninstall netifaces
pip install netifaces
```

## Step 11: Install anafi_ros

Clone the `anafi_ros` repository and build it:

```sh
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone -b ros1 https://github.com/andriyukr/anafi_ros.git
sudo chmod -R 777 anafi_ros/
cd ~/catkin_ws
catkin_make
```
