import os
import numpy as np
import open3d


class Visualizer:
    dataset_path = 'Dataset'
    map_label_to_rgb = {
        1: [0, 255, 0],
        2: [0, 0, 255],
        3: [255, 0, 0],
        4: [255, 0, 255],  # purple
        5: [0, 255, 255],  # cyan
        6: [255, 255, 0],  # yellow
    }

    def __init__(self):
        pass

    def visualize(self, obj_category, obj_id):
        # Concat paths
        pts_path = os.path.join(Visualizer.dataset_path, obj_category,
                                'points', obj_id + '.pts')
        label_path = os.path.join(Visualizer.dataset_path, obj_category,
                                  'points_label', obj_id + '.seg')

        # Read point cloud
        point_cloud = open3d.io.read_point_cloud(pts_path, format='xyz')
        print(point_cloud)

        # Read label and map to color
        labels = np.loadtxt(label_path)
        colors = np.array(
            [Visualizer.map_label_to_rgb[label] for label in labels])
        point_cloud.colors = open3d.utility.Vector3dVector(colors)
        open3d.visualization.draw_geometries([point_cloud])


if __name__ == '__main__':
    v = Visualizer()
    v.visualize('O', '33')
