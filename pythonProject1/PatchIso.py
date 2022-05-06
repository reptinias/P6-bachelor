import open3d as o3d
import numpy as np
import time
from tqdm import tqdm

start = time.time()

for i in tqdm(range(12,161)):
    # load 3D mesh
    Cad = o3d.io.read_triangle_mesh("ValidDatasets/NewDataset/LapperCAD/" + str(i) + "/0.stl")

    # poisson disk sampling on the loaded 3D mesh
    CadPCD = Cad.sample_points_poisson_disk(100000, init_factor=5, pcl=None)

    # save
    o3d.io.write_point_cloud("ValidDatasets/PatchPCD/p." + str(i) + ".ply", CadPCD, write_ascii=True, compressed=False, print_progress=True)


end = time.time()
print(end)