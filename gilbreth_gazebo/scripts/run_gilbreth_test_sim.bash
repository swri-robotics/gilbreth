#!/bin/bash

export GAZEBO_MODEL_PATH="$(rospack find gilbreth_gazebo)/models"
echo $GAZEBO_MODEL_PATH

roslaunch gilbreth_gazebo gilbreth_gazebo.launch
