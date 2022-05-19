import os
import random
from random import randrange
from IPython.display import clear_output
import numpy as np
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.optim as optim
import torch.utils.data
import torchvision.datasets as dset
import torchvision.transforms as transforms
import torchvision.utils as vutils
from torch.autograd import Variable
from datasets import PartDataset
from pointnet import PointNetCls
import torch.nn.functional as F
import matplotlib.pyplot as plt
import open3d as o3
#import download

if torch.cuda.is_available():
    import torch.backends.cudnn as cudnn
# General parameters
NUM_POINTS = 10000
MODEL_PATH = 'cls/cls_model_4.pth'
DATA_FOLDER = 'Test_Dataset'

# download dataset and pre-trained model
#download.download_contents()
# Create dataset object
test_dataset_seg = PartDataset(
    root=DATA_FOLDER,
    train=False,
    classification=False,
    npoints=NUM_POINTS)

# Problem ontology
classes_dict = {'O': 0, 'I': 1}
# Create the classification network from pre-trained model
classifier = PointNetCls(k=len(classes_dict.items()), num_points=NUM_POINTS)

classifier.load_state_dict(torch.load(MODEL_PATH, map_location='cpu'))

classifier.eval()


# Simple point cloud coloring mapping
def read_pointnet_colors(seg_labels):
    map_label_to_rgb = {
        0: [0, 255, 0],
        1: [0, 0, 255],
        2: [255, 0, 0],
        3: [255, 0, 255],  # purple
        4: [0, 255, 255],  # cyan
        5: [255, 255, 0],  # yellow
    }
    colors = np.array([map_label_to_rgb[label] for label in seg_labels])
    return colors


# Three.js based visualizer
visualizer = o3.visualization.Visualizer()

# Basic inference and visualization loop
MAX_SAMPLES = 20
for samples in range(MAX_SAMPLES):
    random_index = randrange(len(test_dataset_seg))
    print('[Sample {} / {}]'.format(random_index, len(test_dataset_seg)))

    # clean visualization
    visualizer.clear_geometries()
    clear_output()

    # get next sample
    point_set, seg = test_dataset_seg.__getitem__(random_index)

    # create cloud for visualization
    cloud = o3.geometry.PointCloud()

    ##### point_set might be using expontential values on first iteration which breaks the vizualization.
    #####quick fix cast firts idx to string and check if it contains value "e" and then break loop and increment max_samples by 1

    cloud.points = o3.utility.Vector3dVector(point_set)
    cloud.colors = o3.utility.Vector3dVector(read_pointnet_colors(seg.numpy()))

    # perform inference in GPU
    points = Variable(point_set.unsqueeze(0))
    points = points.transpose(2, 1)
    if torch.cuda.is_available():
        points = points.cuda()
    pred_logsoft, _ = classifier(points)

    # move data back to cpu for visualization
    pred_logsoft_cpu = pred_logsoft.data.cpu().numpy().squeeze()
    pred_soft_cpu = np.exp(pred_logsoft_cpu)
    pred_class = np.argmax(pred_soft_cpu)

    # let's visualize the input sample
    visualizer.add_geometry(cloud)
    visualizer.create_window()

    # Visualize probabilities
    plt.xticks(list(classes_dict.values()), list(classes_dict.keys()), rotation=90)
    plt.xlabel('Classes')
    plt.ylabel('Probabilities')
    plt.plot(list(classes_dict.values()), pred_soft_cpu)
    plt.show()

    input('Your object is a [{}] with probability {:0.3}. Press enter to continue!'
          .format(list(classes_dict.keys())[pred_class], pred_soft_cpu[pred_class]))