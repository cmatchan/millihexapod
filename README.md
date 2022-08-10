# Millihexapod

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

2. Enter the image bash shell.<br/>
`$ docker exec -it millihex bash`