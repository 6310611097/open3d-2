import open3d as o3d

pcd = o3d.io.read_point_cloud("..\..\ply_file\kota_circuit2.ply")

# set crop boundary
min_x,min_y,min_z = -50,-100,-100
max_x,max_y,max_z = 50,0,-75

crop_area = o3d.geometry.AxisAlignedBoundingBox(min_bound=[min_x,min_y,min_z],
                                                max_bound=[max_x,max_y,max_z])

# downsample
# downpcd = pcd.voxel_down_sample(voxel_size=1)

# crop the area
somewhere = pcd.crop(crop_area)

# estimate normal vetor
somewhere.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

# write ply file name sample.ply
o3d.io.write_point_cloud("sample.ply",
                             somewhere,
                             format='auto',
                             write_ascii=False,
                             compressed=False,
                             print_progress=False
                             )

# plain cropping
plane_model, inliers = somewhere.segment_plane(distance_threshold=0.28,
                                         ransac_n=3,
                                         num_iterations=1000000)

# [a, b, c, d] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# select the plain surface
plain = somewhere.select_by_index(inliers)
# outlier_cloud = something.select_by_index(inliers, invert=True)
# outlier_cloud.paint_uniform_color([1.0, 0, 0])

o3d.visualization.draw_geometries([plain],
                                  zoom=1.5,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])
