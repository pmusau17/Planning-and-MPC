FROM ros:melodic-robot
RUN apt-get update && apt-get install apt-transport-https 
RUN apt-get update &&  apt-get install -y ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-ackermann-msgs ros-melodic-joy
RUN apt-get update &&  apt-get install -y ros-melodic-teb-local-planner ros-melodic-move-base ros-melodic-navigation ros-melodic-hector-slam ros-melodic-driver-common ros-melodic-actionlib


# try eigen three installation
RUN apt-get update && apt-get install -y ros-melodic-pcl-conversions ros-melodic-pcl-msgs ros-melodic-pcl-ros libeigen3-dev libproj-dev libmove-base-msgs-dev
RUN ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
#RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make'


# Install OSQP
RUN git clone --recursive https://github.com/oxfordcontrol/osqp
WORKDIR osqp 
RUN mkdir build
WORKDIR build 
RUN cmake -G "Unix Makefiles" .. && cmake --build .
RUN cmake --build . --target install
WORKDIR ../../

# Install Osqp Eigen
RUN git clone https://github.com/robotology/osqp-eigen.git
WORKDIR osqp-eigen
RUN mkdir build 
WORKDIR build 
RUN cmake ../ && make && make install 

WORKDIR ../../


# Install Catch2

RUN git clone https://github.com/catchorg/Catch2.git
WORKDIR Catch2
RUN cmake -Bbuild -H. -DBUILD_TESTING=OFF && cmake --build build/ --target install 
WORKDIR ..

RUN mkdir -p ~/catkin_ws/src
ENV OsqpEigen_DIR=/osqp-eigen

