import open3d as o3d
import numpy as np
import os

saveMesh = False
algorithmIndex = 2
alpha = 0.3

# load point cloud
pcd = o3d.io.read_point_cloud("20211007_142026_012345_gt.ply")

o3d.visualization.draw_geometries([pcd],
                                  #zoom = 0.3412,
                                  #front = [0.4257, -0.2125, -0.8795],
                                  #lookat = [2.6182, 2.0475, 1.532],
                                  #up = [-0.0694, -0.9768, 0.2024]
                                  )

# voxel downsample af point cloud
downpcd = pcd.voxel_down_sample(voxel_size=0.05)

# normal estimation
downpcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))

# ball-pivoting algorithm for mesh estimation vvvv
# calculate the ball radius
distances = downpcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

if algorithmIndex == 0:
    # create mesh using alpha shapes
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(downpcd, alpha)

if algorithmIndex == 1:
    # create mesh using ball pivoting
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(downpcd, o3d.utility.DoubleVector([radius, radius*3]))

if algorithmIndex == 2:
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(downpcd, depth=9)

# visuellisering af point cloud
#o3d.visualization.draw_geometries([mesh],
#                                  zoom = 0.3412,
#                                  front = [0.4257, -0.2125, -0.8795],
#                                  lookat = [2.6182, 2.0475, 1.532],
#                                  up = [-0.0694, -0.9768, 0.2024],
#                                  point_show_normal = False
#                                  )

# export mesh
def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods={}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        o3d.io.write_triangle_mesh(path+"lod_"+str(i)+extension, mesh_lod)
        mesh_lods[i]=mesh_lod
    print("generation of "+str(i)+" LoD successful")
    return mesh_lods

if saveMesh == True:
    my_lods = lod_mesh_export(mesh, [100000, 8000,800,300], ".ply", "C:/Users/mikael/PycharmProjects/pythonProject1/radius")
#o3d.visualization.draw_geometries([my_lods])

# point amount
print(downpcd)
# point positions
print(np.asarray(downpcd.points))
# point normals
print(np.asarray(downpcd.normals))