import open3d as o3d
import numpy as np
import time
import shutil
from tqdm import tqdm
from os import listdir
from os.path import isfile, join
import os

path = "GroundTruthPCD/"
pointClouds = [f for f in listdir(path) if isfile(join(path, f))]

for i in tqdm(pointClouds):
    pcd = o3d.io.read_point_cloud(path + i)
    o3d.visualization.draw_geometries([pcd])
    profile = input("Beam profile: ")

    fileName = os.path.splitext(i)[0]

    ptsFile = "GroundTruthPTS/" + fileName + ".pts"
    segFile = "SegFiles/" + fileName + ".seg"

    if os.path.isdir("Dataset/" + profile):
        # Send ptsFile og segFile to folder
        pointsFolder = "Dataset/" + profile + "/points/"
        points_labelFolder = "Dataset/" + profile + "/points_label/"

        shutil.move(ptsFile, pointsFolder + str(fileName) + ".pts")
        shutil.move(segFile, points_labelFolder + str(fileName) + ".seg")
    else:
        os.mkdir("Dataset/" + profile)
        os.mkdir("Dataset/" + profile + "/points")
        os.mkdir("Dataset/" + profile + "/points_label")

        pointsFolder = "Dataset/" + profile + "/points/"
        points_labelFolder = "Dataset/" + profile + "/points_label/"

        shutil.move(ptsFile, pointsFolder + str(fileName) + ".pts")
        shutil.move(segFile, points_labelFolder + str(fileName) + ".seg")

