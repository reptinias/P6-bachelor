import open3d as o3d
import numpy as np
import time
import shutil
from tqdm import tqdm
from os import listdir
from os.path import isfile, join

start = time.time()

for i in tqdm(range(0)):
    # load 3D mesh
    Cad = o3d.io.read_triangle_mesh("ValidDatasets/NewDataset/LapperCAD/" + str(i) + "/0.stl")
    o3d.visualization.draw_geometries([Cad])

for i in tqdm(range(0)):
    # load 3D mesh
    Cad = o3d.io.read_triangle_mesh("ValidDatasets/NewDataset/LapperCAD/" + str(i) + "/0.stl")

    # poisson disk sampling on the loaded 3D mesh
    CadPCD = Cad.sample_points_poisson_disk(100000, init_factor=5, pcl=None)

    # load in a point cloud
    pcd = o3d.io.read_point_cloud("ValidDatasets/NewDataset/ValidPC/" + str(i) + ".ply")

    # calculate the distance between two point, if they are the closest point set
    pcdist = np.asarray(pcd.compute_point_cloud_distance(CadPCD))
    ind = np.where(pcdist > 20)[0]
    disPCD = pcd.select_by_index(ind)

    # save
    o3d.io.write_point_cloud("ValidDatasets/CleanDataset/" + str(i) + ".ply", disPCD, write_ascii=True, compressed=False, print_progress=True)

for i in tqdm(range(161)):
    pcd = o3d.io.read_point_cloud("ValidDatasets/CleanDataset/" + str(i) + ".ply")
    o3d.visualization.draw_geometries([pcd])

end = time.time()
print(end - start)