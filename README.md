
# Setting up the Environment

## First create a container with a ubuntu 20.04 image
```sh
# Update and install dependencies
sudo apt update
sudo apt install software-properties-common
sudo apt install curl
sudo apt update

```sh
# Install Python 3.9
sudo apt install python3.9 python3.9-distutils python3.9-venv
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 2
sudo update-alternatives --config python3 # Select the python 3.9 version

```sh
# Install pip for Python 3.9
sudo curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python3.9 get-pip.py

```sh
# Install Olympe
pip3 install https://github.com/Parrot-Developers/olympe/releases/download/v7.5.0/parrot_olympe-7.5.0-py3-none-manylinux_2_27_x86_64.whl # This is the recommended version
echo "export PATH=\"$HOME/.local/bin:$PATH\"" >> ~/.bashrc
source ~/.bashrc

```sh
# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

```sh
# Fix netifaces issue
sudo pip uninstall netifaces
pip install netifaces

```sh
# Install anafi_ros
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone -b ros1 https://github.com/andriyukr/anafi_ros.git
sudo chmod -R 777 anafi_ros/
cd ~/catkin_ws && catkin_make
