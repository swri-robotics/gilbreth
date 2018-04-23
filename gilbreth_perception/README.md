1.Save gilbreth_perception folder under your-catkin-workspace/src/gilbreth.
2.Under your-catkin-workspace run command "catkin_make".
3.Run command "roslaunch gilbreth_perception gilbreth_perception.launch".

For using convolutional neural network (instead of the correspondence grouping) in the recognition node:

1. Make sure your computer has Nvidia GPU;
2. Install voxnet at https://github.com/dimatura/voxnet;
- Install all dependent packages as instructed on the webpage
- Remember to add voxnet path into your PYTHONPATH by command:
  ‘export PYTHONPATH=$PYTHONPATH:you-path-to-voxnet/voxnet’
- Verify voxnet has been installed successfully by ‘import voxnet’ in python, make sure no error occurs
- If you have error like ‘CudaNdarrayType only supports dtype float32 for now’ , solution is here:
  https://github.com/dimatura/voxnet/issues/5
3. Save your neural network weights into src/gilbreth/gilbreth_perception/config/
4. Run command "roslaunch gilbreth_perception gilbreth_perception_cnn.launch"
