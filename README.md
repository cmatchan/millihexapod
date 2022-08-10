# Millihexapod

## Requirements

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
and create an account. Docker allows easy portability of complex packages and
makes it easy for developers on different Operating Systems to work on the same
project without worrying about dependency incompatabilities or installation.

2. Set up X11 Forwarding to enable native graphics when running simulations in
Gazebo. I use [VcXsrv](https://sourceforge.net/projects/vcxsrv/) for Windows.

## Getting started

1. In a new shell, pull the image from Dockerhub.<br/>
`$ docker pull cmatchan/millihexapod:gait-param`

2. Start the image in a new container in interactive mode.<br/>
`$ docker run --name millihex -it cmatchan/millihexapod:gait-param bash`<br/>
This command enters the image's Linux bash shell with ROS installed.

3. To stop the container, execute the following command.<br/>
`$ docker stop millihex`

## Running the demo

1. Start the Docker container again.<br/>
`$ docker start millihex`

2. Enter the conteiner's Linux shell.<br/>
`$ docker exec -it millihex bash`

3. Enable X11 Forwarding on your computer. If you use VcXsrv, click the
`config.xlaunch` file in the `catkin_ws/src/millihexapod` directory.

3. In the container shell, spawn a Millihex robot in Gazebo.<br/>
`$ roslaunch millihexapod spawn_millihex.launch`

4. Open a new terminal and repeat steps 1. and 2. to enter a second container
shell.<br/>
`$ docker start millihex`<br/>
`$ docker exec -it millihex bash`

5. In the second container shell, run the demo script.<br/>
`$ rosrun millihexapod demo.py`