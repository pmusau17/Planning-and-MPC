#! /bin/bash

#run the docker container
docker container run --rm --runtime=nvidia -it -e DISPLAY --net=host --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix -d simulator bash