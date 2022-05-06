import open3d as o3d
import numpy as np
import time
import shutil
from tqdm import tqdm
from os import listdir
from os.path import isfile, join
import os

path = "ValidDatasets/CleanDataset/"
index = 0

onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

for i in tqdm(onlyfiles):
    pcd = o3d.io.read_point_cloud("ValidDatasets/CleanDataset/" + i)
    o3d.visualization.draw_geometries([pcd])
    old_name = path + i
    new_name = path + str(index) + "." + input("Beam profile: ") + ".ply"

    os.rename(old_name, new_name)

    index += 1
