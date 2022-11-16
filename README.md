# Lab_7

## Introduction

The purpose of this lab is to familiarize you with the UR3 robot and the tools we have for making it do useful and/or interesting things. Additionally, you will cause the end point of the robot arm to move in both a square and circle in a vertical plane and in a horizontal plane. The Horizontal Plane is the plane perpendicular to Z-Axis and the Vertical plane is the plane perpendicular to either X or Y axes. Next, you will cause the end point of the arm to move in the largest possible square that is not in the horizontal plane. Finally, you will move the end point of the arm through this largest square as quickly as possible.

**NOTE**: Throughout the course you will **first** create a successful simulation of the desired arm movement in Gazebo. Only after getting this simulation approved by the lab staff will you implement it on the actual arm. This is a very important safety measure.

Throughout this part of the course, you will use a collection of tools that we have provided. These include Docker containers, ROS, Moveit, and Gazebo. The basics of these tools are explained in the lectures. These tools are already loaded on the lab computers.

All the tools you need to do this lab are in a Docker container. Docker containers are built on images which are built from dockerfiles. Your system in the lab has the docker image with all the required tools such as ROS and Moveit! Installed. You just need to build a container using that image and work inside the container. To save the work, you will volume map a directory from the host pc to the docker container. Any changes you make in the mapped directory inside the container will reflect in the directory in the host pc. To learn more about docker visit [docker documentation](https://docs.docker.com/).

**Volume Map**: Docker Containers are destroyed when you exit the container, which means all the data will be lost. Volume mapping is used to save the important data before destroying the container. A directory from host pc is mapped to a directory in the container. Changes made in the mapped directory in the docker container is reflected in the host pc. Save the important data in the mapped directory inside the container.

**The commands given in each step below are meant to be copied and pasted in the terminal**.

If you want to work on your own computer, install docker and portainer using [this page](https://github.com/ENRE467/Getting_Started/wiki/Installing-Docker-and-Portainer) and build a docker image using [this page](https://github.com/ENRE467/Getting_Started/wiki/Building-a-Docker-Image)

## Steps

1. Create your own folder on the lab machine so you can save your work. Also, just to be sure, upload your work to your Github account before you leave the lab. Do this in the host pc and not inside the Docker container as `git push` and `git pull` commands will not work inside the container. This is because your git repositories does not exist inside the docker container.

2. Open a terminal window by pressing `Ctrl + Alt + T`. In the terminal window, navigate to your folder using cd command. Now, run the following command to clone the repository for Lab 7:

```console
git clone https://github.com/ENRE467/Lab_7.git
```

3. Navigate to the `Lab_7/src` directory in your folder using cd command.

4. Visual Studio Code (VSC) is already installed in your systems in the lab. You will use VSC to write code in this part of the course. To open the src directory in VSC, run the following command:

```console
code .
```

5. Run the following command so that you can see the GUI applications from docker container in the screen of the host pc:

```console
xhost +local:docker
```

6. Now, you will create a docker container based on the `ur3e_image` image which is already on your lab computer and volume map the src directory in the host pc to the src directory in the docker container. To do that, enter the following command (Make sure that you are in the Lab_7/src directory inside the terminal before running this command):

```console
docker run -it --rm --name UR3Container --net=host --pid=host --privileged --env="DISPLAY=$DISPLAY" --volume="$PWD:/home/${USER}/workspace/src" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ur3e_image:latest
```

7. Now, you are in the workspace directory in the docker container. This is your catkin workspace. Check that the `~/workspace/src` directory contains the files from the `Lab_7/src` directory in your host pc by using the command `ls ~/workspace/src`. This will list all the files in your src folder. Now, if everything seems good, the first thing you do is build your catkin packages. To do that, first go to the workspace directory (if you are not already there) using the command `cd ~/workspace`. To build the workspace, execute the following command:

```console
catkin build
```

After the packages are built, you need to source them so that you can use them in the current terminal window. Run the following command to do that:

```console
source ~/workspace/devel/setup.bash
```

 To build projects in ROS, it is advised to follow the specific directory structure. place all you code files in the src folder of the catkin project. see below for reference.

```console
┌──────────────────────────┐             ┌──────────────────────────┐
│ ProjectDir               │             │ catkin_ws                │
│ │                        │             │   │                      │
│ └────src                 │             │   └──src                 │
│      │                   │             │      │                   │
│      ├─catkin_project_1  │             │      ├─catkin_project_1  │
│      │                   │             │      │                   │
│      ├─catkin_project_2  ├────────────►│      ├─catkin_project_2  │
│      │        x          │             │      │        x          │
│      │        x          │ Volume Map  │      │        x          │
│      │        x          │             │      │        x          │
│      │        x          │             │      │        x          │
│      └─catkin_project_N  │             │      └─catkin_project_N  │
│                          │             │                          │
└──────────────────────────┘             └──────────────────────────┘
```

8. Tmux is a tool which is used to split a terminal window into multiple terminals. Tmux is already installed in your docker container. To split the terminal vertically, type tmux and press enter, this will open the current terminal with tmux, then click on the terminal you want to split and press `Ctrl + A` to select that terminal and press `V` to split it vertically. To split the terminal horizontally, click on the terminal you want to split and press `Ctrl + A` to select it and then press `B` to split it horizontally to do it manually. 
An example command to split into four terminals using terminal commands is below:

```console
tmux new-session \; \split-window -v \; \split-window -h \; \select-pane -t 1 \; \split-window -h
```

9. Run the following command to start gazebo with the UR3e arm in it:

```console
roslaunch ur3e_setup ur3e_gazebo.launch z_height:=0.8
```

`z_height` is the height at which the robot is spawned in Gazebo.

10. In a different terminal window, run the following command to start Moveit! functionality and start RViz:

```console
roslaunch ur3e_setup ur3e_moveit.launch
```

11. Run the following command to add collision objects:

```console
roslaunch ur3e_setup setup.launch
```

12. The `moveit_tutorial` package has sample code for performing three tasks: 1. Move the robot to a joint goal, 2. Move the robot to a pose goal and 3. Move the robot from one point to another in a cartesian path. You can refer to the `tutorial.cpp` in the `moveit_tutorial` package for the sample code. This sample code uses the helper functions from `moviet_wrapper` package. In a new terminal, run the following command to run this sample code:

``` console
rosrun moveit_tutorial tutorial
```

13. You will use these helper functions in your code to move your robot in square and circle trajectories. A package for this lab is provided to you and the name of this package is `ur3e_trajectory`. Add your code to the files `square.cpp` and `circle.cpp` for square and circle trajectories. 

Run the following command to run your code for square or circle trajectories:

```console
rosrun ur3e_trajectory square 
```

Replace square with circle if you want to run your circle code.

14. You need to calculate the error between the trajectory followed by your robot and the desired trajectory. To do this, you have to record the end effector positions while your robot traces the trajectory. The `RecordPose.cpp` file contains the code to record end effector positions at the rate of 2 Hz. It starts recording poses when the boolean parameter `record_pose` turns true. You have to set the value of this parameter to true before executing the trajectory and set it to false after trajectory executions. Look at the end of `tutorial.cpp` file in the `moveit_tutorial` package for sample implementation. The boolean parameter `record_pose` needs to be loaded to parameter server and the `RecordPose.cpp` program will look for that parameter from the parameter server. Run the following command to load the parameter:

```console
roslaunch ur3e_trajectory load_params.launch
```
Edit the string variable `out_path` in the `RecordPose.cpp` file to the destination where you want to save your end effector poses. After this is done, Run the following command at the same time you run your code for square or circle trajectory:

```console
rosrun ur3e_trajectory RecordPose
```

You can use the generated csv file of the end effector poses to plot the followed trajectory against the ideal trajectory.

15. After you are done with your simulation. You can run your code on the real UR3e arm. Ask one of the Teaching Assistants to help you.
