import os
import open3d as o3d
import numpy as np
import time
import shutil
from os import listdir
from os.path import isfile, join

groundTruthList = []

# Load all folders in dataset
AllFiles = os.listdir("InropaFiles")

def groundTruth(skipIndex=None):
    os.mkdir("GroundTruthPCD")
    os.mkdir("GroundTruthPTS")

    print(AllFiles)
    # Loop through all folders
    for i, folder in enumerate(AllFiles):
        if skipIndex >= 50:
            continue
        # Load all scans in the folder
        scans = os.listdir("InropaFiles/" + str(folder) + "/Scans/")

        # Create the array that will contain the combined vertices of the stl models
        pcdArray = np.array([[0, 0, 0]])

        # Find the vertices for each stl file and save the vertices in an array
        # Concatenate the saved vertices and combined vertices array
        for x in scans:
            foundVertices = np.asarray(o3d.io.read_triangle_mesh("InropaFiles/" + str(folder) + "/Scans/" + x).vertices)
            pcdArray = np.concatenate((pcdArray, foundVertices), axis=0)

        # Create an empty point cloud object and add the vertices to the point cloud as positions
        pcd = o3d.geometry.PointCloud()
        pcdArray = np.delete(pcdArray, 0, axis=0)
        pcd.points = o3d.utility.Vector3dVector(pcdArray)

        CADPCD = np.array([[0, 0, 0]])
        CADFiles = [f for f in listdir("InropaFiles/" + str(folder) + "/CADs") if isfile(join("InropaFiles/" + str(folder) + "/CADs", f))]
        for x in CADFiles:
            # Read the ground truth Cad model
            Cad = o3d.io.read_triangle_mesh("InropaFiles/" + str(folder) + "/CADs/" + x)
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
            o3d.io.write_point_cloud("GroundTruthPCD/" + str(folder) + ".ply", endPCD, write_ascii=True, compressed=False, print_progress=True)
            o3d.io.write_point_cloud("GroundTruthPTS/" + str(folder) + ".pts", endPCD, write_ascii=True, compressed=False, print_progress=True)
            print("point cloud and CAD saved with acurracy of " + str(removePer))
        else:
            print("point cloud and CAD discarded")
        skipIndex += 1

def beamIso(pointCloud, skipIndex=None):
    os.mkdir("SegFiles")
    patchFolders = os.listdir("GroundTruthPCD")
    for i, folder in enumerate(patchFolders):
        if skipIndex >= 50:
            continue

        try:
            # load 3D mesh
            Cad = o3d.io.read_triangle_mesh("LapperCAD/" + str(os.path.splitext(folder)[0]) + ".stl")
            print(os.path.splitext(folder)[0])

            # poisson disk sampling on the loaded 3D mesh
            CadPCD = Cad.sample_points_poisson_disk(10000, init_factor=5, pcl=None)

            # load in a point cloud
            pcd = o3d.io.read_point_cloud("GroundTruthPCD/" + str(folder))

            # calculate the distance between two point, if they are the closest point set
            pcdist = np.asarray(pcd.compute_point_cloud_distance(CadPCD))

            f = open("SegFiles/" + str(os.path.splitext(folder)[0]) + ".seg", "x")

            for i in pcdist:
                if i > 20:
                    f.write(str(1) + "\n")
                    # beam
                else:
                    f.write(str(2) + "\n")
                    # patch

            f.close()
            ind = np.where(pcdist > 20)[0]
            disPCD = pcd.select_by_index(ind)

            print("Jeg er her")
            #lapDist = np.asarray(pcd.compute_point_cloud_distace(CadPCD))
            index = np.where(pcdist < 10)[0]
            lapPCD = pcd.select_by_index(index)

            skipIndex += 1

        except:
            print("jeg ved ikke hvad jeg laver")
            # load in a point cloud
            pcd = np.asarray(o3d.io.read_point_cloud("GroundTruthPCD/" + str(folder)).points)
            f = open("SegFiles/" + str(os.path.splitext(folder)[0]) + ".seg", "x")
            for i in pcd:
                f.write(str(0) + "\n")



groundTruth(0)
beamIso(groundTruthList, 0)