# read in bg_frame.jpg and frame_depth.png

import cv2
import numpy as np
from matplotlib import pyplot as plt
import open3d as o3d
import argparse
import glob
import os

def get_intrinsic_matrix(frame, imwidth, imheight):
    if frame is None:
        # default values for Intel RealSense D435
        fx, fy = 385.7314758300781, 385.2828369140625
        ppx, ppy = 322.3908996582031, 239.84091186523438
        return o3d.camera.PinholeCameraIntrinsic(imwidth, imheight, fx, fy, ppx, ppy)
    
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = o3d.camera.PinholeCameraIntrinsic(imwidth,  imheight, intrinsics.fx,
                                            intrinsics.fy, intrinsics.ppx,
                                            intrinsics.ppy)
    return out

def get_images(folder):
     # Get the folder path from the arguments
    folder_path = folder

    # Ensure the folder path ends with a slash
    if not folder_path.endswith('/'):
        folder_path += '/'

    # Get all files matching *_color.png in the specified folder
    all_color_files = glob.glob(os.path.join(folder_path, '*_color.png'))
    all_depth_files = glob.glob(os.path.join(folder_path, '*_depth.png'))
    filtered_files = [file for file in all_color_files if 'bg_color.png' not in os.path.basename(file)]

    color_np = cv2.imread(filtered_files[0])
    depth_np = cv2.imread(all_depth_files[0], cv2.IMREAD_UNCHANGED)

    return color_np, depth_np

def output_point_cloud(folder, pcd):
    # Create the output directory if it doesn't exist
    output_folder = folder.replace('testing_data', 'output')
    os.makedirs(output_folder, exist_ok=True)
    print('Saving point cloud to', output_folder)
    # Save point cloud
    o3d.io.write_point_cloud(os.path.join(output_folder, 'pc.ply'), pcd)

def main(folder):
    color_np, depth_np = get_images(folder)

    print(color_np.shape, depth_np.shape)
    
    # bgr -> rgb
    color_np = cv2.cvtColor(color_np, cv2.COLOR_BGR2RGB)

    imwidth, imheight, channel = color_np.shape
    color_image = o3d.geometry.Image(color_np)
    depth_image = o3d.geometry.Image(depth_np)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image,
        convert_rgb_to_intensity=False) # false = colored   
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, get_intrinsic_matrix(None, imwidth, imheight))

    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    # o3d.visualization.draw_geometries([pcd])

    output_point_cloud(folder, pcd)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="folder name")

    args = parser.parse_args()

    main(args.folder)