#! /bin/bash

#run the docker container
docker run -it --rm --runtime=nvidia --privileged --net=host \
    -e DISPLAY=:0 \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/catkin_ws/src:rw \
    --name=planning planning:0.0.1 bash