import os
import open3d as o3d
import numpy as np
import time
import shutil
from tqdm import tqdm
from os import listdir
from os.path import isfile, join

groundTruthList = []

# Load all folders in dataset
AllFiles = os.listdir("Dataset")

def groundTruth(skipIndex=None):
    shutil.rmtree('GroundTruth')
    shutil.rmtree('CleanDataset')
    shutil.rmtree('CleanPCD')
    os.mkdir("GroundTruth")
    print(AllFiles)
    # Loop through all folders
    for i, folder in tqdm(enumerate(AllFiles)):
        if skipIndex >= 20:
            continue

        print(folder)
        # Load all scans in the folder
        scans = os.listdir("dataset/" + str(folder) + "/Scans/")

        # Create the array that will contain the combined vertices of the stl models
        pcdArray = np.array([[0, 0, 0]])

        # Find the vertices for each stl file and save the vertices in an array
        # Concatenate the saved vertices and combined vertices array
        for x in scans:
            foundVertices = np.asarray(o3d.io.read_triangle_mesh("Dataset/" + str(folder) + "/Scans/" + x).vertices)
            pcdArray = np.concatenate((pcdArray, foundVertices), axis=0)

        # Create an empty point cloud object and add the vertices to the point cloud as positions
        pcd = o3d.geometry.PointCloud()
        pcdArray = np.delete(pcdArray, 0, axis=0)
        pcd.points = o3d.utility.Vector3dVector(pcdArray)

        CADPCD = np.array([[0, 0, 0]])
        CADFiles = [f for f in listdir("Dataset/" + str(folder) + "/CADs") if isfile(join("Dataset/" + str(folder) + "/CADs", f))]
        for x in CADFiles:
            # Read the ground truth Cad model
            Cad = o3d.io.read_triangle_mesh("Dataset/" + str(folder) + "/CADs/" + x)
            # Convert the Cad model into a point cloud
            CadPCD = Cad.sample_points_poisson_disk(100000, init_factor=5, pcl=None)
            CADPCD = np.concatenate((CADPCD, CadPCD.points), axis=0)

        GroundPCD = o3d.geometry.PointCloud()
        CADPCD = np.delete(CADPCD, 0, axis=0)
        GroundPCD.points = o3d.utility.Vector3dVector(CADPCD)

        # Calculate distances between pcd and CadPCD.
        pcdist = pcd.compute_point_cloud_distance(GroundPCD)

        # pcdist is an Open3d object, we need to convert it to a numpy array to access the data
        pcdist = np.asarray(pcdist)

        # Remove point that is further than x away from closest neighbor
        ind = np.where(pcdist < 20)[0]
        endPCD = pcd.select_by_index(ind)

        # Calculate point removed in percentage
        removePer = (len(endPCD.points) / len(pcdArray)) * 100

        if removePer > 90:
            o3d.io.write_point_cloud("GroundTruth/" + str(folder) + ".ply", endPCD, write_ascii=True, compressed=False, print_progress=True)
            print("point cloud and CAD saved with acurracy of " + str(removePer))
        else:
            print("point cloud and CAD discarded")
        skipIndex += 1

def beamIso(pointCloud, skipIndex=None):
    os.mkdir("CleanDataset")
    os.mkdir("CleanPCD")
    patchFolders = os.listdir("GroundTruth")
    for i, folder in tqdm(enumerate(patchFolders)):
        if skipIndex >= 20:
            continue

        try:
            # load 3D mesh
            Cad = o3d.io.read_triangle_mesh("LapperCAD/" + str(os.path.splitext(folder)[0]) + ".stl")

            # poisson disk sampling on the loaded 3D mesh
            CadPCD = Cad.sample_points_poisson_disk(10000, init_factor=5, pcl=None)

            # load in a point cloud
            pcd = o3d.io.read_point_cloud("GroundTruth/" + str(folder))

            # calculate the distance between two point, if they are the closest point set
            pcdist = np.asarray(pcd.compute_point_cloud_distance(CadPCD))
            ind = np.where(pcdist > 20)[0]
            disPCD = pcd.select_by_index(ind)

            skipIndex += 1

            # save
            o3d.io.write_point_cloud("CleanPCD/" + str(os.path.splitext(folder)[0]) + ".ply", disPCD, write_ascii=True, compressed=False, print_progress=True)
            o3d.io.write_point_cloud("CleanPCD/p." + str(os.path.splitext(folder)[0]) + ".ply", CadPCD, write_ascii=True, compressed=False, print_progress=True)
            o3d.io.write_point_cloud("CleanDataset/" + str(os.path.splitext(folder)[0]) + ".pts", disPCD, write_ascii=True, compressed=False, print_progress=True)
            o3d.io.write_point_cloud("CleanDataset/p." + str(os.path.splitext(folder)[0]) + ".pts", CadPCD, write_ascii=True, compressed=False, print_progress=True)
        except:
            # load in a point cloud
            pcd = o3d.io.read_point_cloud("GroundTruth/" + str(folder))
            o3d.io.write_point_cloud("CleanPCD/" + str(os.path.splitext(folder)[0]) + ".ply", pcd, write_ascii=True, compressed=False, print_progress=True)
            o3d.io.write_point_cloud("CleanDataset/" + str(os.path.splitext(folder)[0]) + ".pts", pcd, write_ascii=True, compressed=False, print_progress=True)


groundTruth(0)
beamIso(groundTruthList, 0)