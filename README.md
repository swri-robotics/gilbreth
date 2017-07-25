# Gilbreth
robotic product handling simulation 

[Lillian Moller Gilbreth](https://en.wikipedia.org/wiki/Lillian_Moller_Gilbreth)  was an American psychologist and industrial engineer.

### Run SDF to URDF converter
1. Download SDF parser script into your catkin workspace
  ```
  cd $(catkin locate -s)
  git clone https://github.com/jrgnicho/pysdf.git
```

2. Setup Global variables
  ```
  export GAZEBO_MODEL_PATH=$(catkin locate -s)/gilbreth/gilbreth_gazebo/models
  export MESH_WORKSPACE_PATH=$(catkin locate)
  ```

3. Run parser
  ```
  roscd gilbreth_gazebo
  rosrun pysdf sdf2urdf.py models/environment/model.sdf my_urdf.urdf
  ```
