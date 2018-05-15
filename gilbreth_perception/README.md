For using Correspondence Grouping method in object recognition:
1.Run command "roslaunch gilbreth_perception gilbreth_perception.launch cnn:=false".

For using convolutional neural network method in object recognition:
1. Make sure your computer has Nvidia GPU;
2. Install voxnet at https://github.com/dimatura/voxnet;
- Install all dependent packages
    Theano http://deeplearning.net/software/theano/
    Lasagne https://github.com/Lasagne/Lasagne
    path.py https://github.com/jaraco/path.py
    scikit-learn http://scikit-learn.org/stable/install.html
- Install voxnet by:
    git clone https://github.com/dimatura/voxnet.git
    cd voxnet
    sudo pip install --editable .
- Verify voxnet has been installed successfully by ‘import voxnet’ in python, make sure no error occurs
    Note that you may need to install old version of theano and a development version of lasagne to make it work by:
       sudo pip install theano==0.9
       sudo pip install --upgrade https://github.com/Lasagne/Lasagne/archive/master.zip
- If you have error like ‘CudaNdarrayType only supports dtype float32 for now’ , solution is here:
  https://github.com/dimatura/voxnet/issues/5
3. Save your neural network weights into src/gilbreth/gilbreth_perception/config/weights.npz
4. Run command "roslaunch gilbreth_perception gilbreth_perception.launch"
