#!/usr/bin/env python
import rospy
from gilbreth_msgs.msg import ObjectVoxel
from gilbreth_msgs.msg import ObjectType
import imp
import time
import numpy as np
import theano
import theano.tensor as T
import lasagne
import voxnet


def make_test_functions(cfg, model):
    l_out = model['l_out']
    batch_index = T.iscalar('batch_index')
    X = T.TensorType('float32', [False]*5)('X')
    y = T.TensorType('int32', [False]*1)('y')
    out_shape = lasagne.layers.get_output_shape(l_out)
    batch_slice = slice(batch_index*cfg['batch_size'], (batch_index+1)*cfg['batch_size'])
    out = lasagne.layers.get_output(l_out, X)
    dout = lasagne.layers.get_output(l_out, X, deterministic=True)
    params = lasagne.layers.get_all_params(l_out)
    softmax_out = T.nnet.softmax(out)
    pred = T.argmax(dout, axis=1)
    X_shared = lasagne.utils.shared_empty(5, dtype='float32')
    dout_fn = theano.function([X], dout)
    pred_fn = theano.function([X], pred)

    tfuncs = {'dout' : dout_fn,
             'pred' : pred_fn,
            }
    tvars = {'X' : X,
             'y' : y,
             'X_shared' : X_shared,
            }
    return tfuncs, tvars


def callback(data):
    object_type_msg = ObjectType()
    start = time.time()
    for i in range(0, dims[0]):
        for j in range(0, dims[1]):
            for k in range(0, dims[2]):
                x[0][0][i][j][k] = data.voxel.data[i*dims[1]*dims[2]+k*dims[2]+j]
    pred = np.argmax(np.sum(tfuncs['dout'](x), 0))
    object_type_msg.type = pred
    end = time.time()
    duration = end-start
    rospy.loginfo("Object Type: %d, CNN prediction time: %f", pred, duration)
    object_type_msg.header.stamp = rospy.Time.now()
    object_type_msg.detection_time = data.detection_time
    object_type_msg.pcd = data.pcd
    pub.publish(object_type_msg)


def recognitioncnn():
    rospy.init_node('recognition_cnn_node', anonymous=True)
    rospy.Subscriber("voxel_data", ObjectVoxel, callback)
    rospy.spin()


if __name__ == '__main__':
    package_path = rospy.get_param('/recognition_cnn/package_path')
    config_module = imp.load_source('config', package_path + '/config/gilbreth_cnn.py')
    cfg = config_module.cfg
    model = config_module.get_model()
    voxnet.checkpoints.load_weights(package_path + '/config/weights.npz', model['l_out'])
    tfuncs, tvars = make_test_functions(cfg, model)
    dims = cfg['dims']
    pub = rospy.Publisher('object_type', ObjectType, queue_size=10)
    x = np.zeros((1,1,)+dims, dtype=np.float32)
    recognitioncnn()
