#! /bin/bash

#run the docker container
docker run -it --rm --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/catkin_ws/src:rw \
    --name=planning planning:0.0.1 bash