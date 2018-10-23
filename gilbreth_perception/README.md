For using Correspondence Grouping method in object recognition:
1.Run command "roslaunch gilbreth_perception gilbreth_perception.launch cnn:=false".

For using convolutional neural network method in object recognition:
1. Make sure your computer has Nvidia GPU;
    lspci | grep VGA
    
   If you have an Nvidia GPU there will be an entry like this:
   02:00.0 VGA compatible controller: NVIDIA Corporation GM107GL [Quadro K620] (rev a2)
   
2. Install voxnet at https://github.com/dimatura/voxnet;
- Install all dependent packages
    Theano version 0.7 http://deeplearning.net/software/theano/
        pip install Theano==0.7 pygpu
    Lasagne version 0.1 https://github.com/Lasagne/Lasagne
        pip install Lasagne==0.1
    path.py https://github.com/jaraco/path.py
        pip install path.py
    scikit-learn http://scikit-learn.org/stable/install.html
        pip install -U scikit-learn
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
