#! /usr/bin/env python
import numpy as np
import open3d as o3d
import sys
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
DATA_DIR = os.path.join(ROOT_DIR,"data")

def cloud_subdivision(x_range, y_range, x_reso, y_reso, cloud):
    return 



if __name__ == "__main__":
    # read data from pcd
    pcd = o3d.io.read_point_cloud(os.path.join(DATA_DIR,"1.pcd"))
    # load data into numpy
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    print(pcd.get_max_bound())
    print(pcd.get_min_bound())
    # subdivide into small region
    max_x = np.max(points[:,1])
    min_x = np.min(points[:,1])
    max_y = np.max(points[:,2])
    min_y = np.min(points[:,2])
    x_range = max_x - min_x
    y_range = max_y - min_y
    points[:,0]=0
    pcd.points = o3d.utility.Vector3dVector(points)
    # visualizaion
    o3d.visualization.draw_geometries([pcd],
                                      lookat=[ -0.00071885912563767005, 0.036710214427356704, 0.098602843157838918 ],
                                      up = [ 0.036150451334005476, 0.064092308544618698, 0.99728898562742052 ],
                                      front = [ -0.99912036372943447, 0.023539986228374905, 0.034704003076456005 ],
                                      zoom = 0.57999999999999985)