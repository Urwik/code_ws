import os
import open3d as o3d

# Get all files in the current directory
files = os.listdir('.')

# Filter the list to include only PLY files
ply_files = [file for file in files if file.endswith('.ply')]

# Read each PLY file and print its size
for ply_file in ply_files:
    pcd = o3d.io.read_point_cloud(ply_file)

    if len(pcd.points) != 20000:
        print(f'File: {ply_file}, Size: {len(pcd.points)}')