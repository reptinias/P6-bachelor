from __future__ import print_function
import argparse
import os
import random
import numpy as np
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data
import torchvision.datasets as dset
import torchvision.transforms as transforms
import torchvision.utils as vutils
from torch.autograd import Variable

#from PointNet import show3d_balls
from PointNet.datasets import PartDataset
from PointNet.pointnet import PointNetDenseCls
import torch.nn.functional as F
import matplotlib.pyplot as plt
#import PointNet.show3d_balls


#showpoints(np.random.randn(2500,3), c1 = np.random.uniform(0,1,size = (2500)))

parser = argparse.ArgumentParser()

parser.add_argument('--model', type=str, default = 'seg/seg_model_2.pth',  help='model path')
parser.add_argument('--idx', type=int, default = 0,   help='model index')



opt = parser.parse_args()
print (opt)

d = PartDataset(root = 'Test_Dataset', class_choice = None, train = True)

idx = opt.idx

print("model %d/%d" %( idx, len(d)))

point, seg = d[idx]
print(point.size(), seg.size())

point_np = point.numpy()



cmap = plt.cm.get_cmap("hsv", 10)
cmap = np.array([cmap(i) for i in range(10)])[:,:3]
gt = cmap[seg.numpy() - 1, :]

classifier = PointNetDenseCls(k = 2)
classifier.load_state_dict(torch.load(opt.model))
classifier.eval()

point = point.transpose(1,0).contiguous()

point = Variable(point.view(1, point.size()[0], point.size()[1]))
pred, _ = classifier(point)
pred_choice = pred.data.max(2)[1]
print(pred_choice)

pred_color = cmap[pred_choice.numpy()[0], :]

