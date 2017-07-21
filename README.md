## Setup

Install the necessary turtlebot packages:

```
sudo apt-get update
sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
```

If you do not have a catkin workspace setup, use the following commands:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

Add a source to the setup scripts to your .bashrc file:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Change to the <catkin_workspace_dir>/src/ and clone the repository:

```
cd catkin_ws/src
git clone https://github.com/AmerSR/turtlebot_indoor_patrol.git
```

## Run
To launch the simulator, open a terminal and type the following:

```
roslaunch turtlebot_indoor_patrol turtlebot_patrol.launch
```

A stage simulator should start showing the turtlebot as a black square, while the objects are shown in red. The turtlebot should avoid any possible collision with these objects.

To send an SOS request from any of the shops:
```
rosrun turtlebot_indoor_patrol sos_request.py <int>
```

where `<int>` is an integer between 1 and 12. This will publish the sos request and the robot navigates to the needed shop immediately.

