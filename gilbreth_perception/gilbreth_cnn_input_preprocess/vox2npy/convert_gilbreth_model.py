
import logging
import random
import numpy as np
import scipy.io
from path import Path
import argparse

import npytar

import binvox_rw

def write(records, fname):
    writer = npytar.NpyTarWriter(fname)
    for (class_id, instance, rot, fname) in records:
        name = '{:03d}.{}.{:02d}'.format(class_id, instance, rot)
        with open (fname,'rb') as f:
            model=binvox_rw.read_as_3d_array(f)
        f.close
        arr = model.data.astype(np.uint8)
        writer.add(arr, name)
    writer.close()


parser = argparse.ArgumentParser()
parser.add_argument('data_dir', type=Path)
args = parser.parse_args()

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s| %(message)s')

#base_dir = Path('~/code/3DShapeNets2/3DShapeNets/volumetric_data').expand()
base_dir = (args.data_dir).expand()

records = {'train': [], 'test': []}

logging.info('Loading .binvox files')
for fname in sorted(base_dir.walkfiles('*.binvox')):
    elts = fname.splitall()
    instance_rot = Path(elts[-1]).stripext()
    instance = instance_rot[instance_rot.find('.')+1:instance_rot.rfind('.')]
    rot = int(instance_rot[instance_rot.rfind('.')+1:])
    split = elts[-2]
    class_id = int(instance_rot[:instance_rot.find('.')])
    records[split].append((class_id, instance, rot, fname))


# just shuffle train set
logging.info('Saving train npy tar file')
train_records = records['train']
random.shuffle(train_records)
write(train_records, 'gilbreth_train.tar')

# order test set by instance and orientation
logging.info('Saving test npy tar file')
test_records = records['test']
test_records = sorted(test_records, key=lambda x: x[2])
test_records = sorted(test_records, key=lambda x: x[1])
write(test_records, 'gilbreth_test.tar')
