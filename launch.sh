#!/bin/bash
docker run  -it \
 -v `pwd`:/home/xiaoyez/ros2_ws/src/ \
 -v `pwd`/docker_history.txt:/root/.bash_history \
 -e DISPLAY=$DISPLAY \
 -h $HOSTNAME \
 --privileged \
 xiaoyezuo/mav:humble