# read in color and depth, read in bg
# subtract bg from color
# modify depth
# gen pc
# save pc

import cv2
import numpy as np
from matplotlib import pyplot as plt
import open3d as o3d
import argparse
import glob
import os
import imutils

displayResize = 900

def preprocess(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #img = cv2.GaussianBlur(img, (47, 47), 0)

    return img

def posprocess(img):
    (T, img) = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)
    # img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
    #    cv2.THRESH_BINARY,11,2)
    img = cv2.dilate(img, None, iterations=42)
    img = cv2.erode(img, None, iterations=42)
    img = cv2.dilate(img, None, iterations=8)

    return img

def findContours(img):
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return cnts

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
    o3d.io.write_point_cloud(os.path.join(output_folder, 'segmented2.ply'), pcd)

def output_rect_image(folder, color_np):
    # Create the output directory if it doesn't exist
    output_folder = folder.replace('testing_data', 'output')
    os.makedirs(output_folder, exist_ok=True)
    print('Saving rect image to', output_folder)
    # Save rect image
    cv2.imwrite(os.path.join(output_folder, 'rect.png'), color_np)

def main(folder):
    color_np, depth_np = get_images(folder)
    
    print(color_np.shape, depth_np.shape)
    
    # bgr -> rgb
    color_np = cv2.cvtColor(color_np, cv2.COLOR_BGR2RGB)

    imwidth, imheight, channel = color_np.shape

    # ------------------------------------------------------------------------------------------------------------------------ #
    bg_path = os.path.join(folder, 'bg_color.png')
    
    try:
        bg_image = cv2.imread(bg_path)
        bg_image = preprocess(bg_image)
    except:
        print("Unable to load background image")
        return

    color_preprocessed = preprocess(color_np)

    mask_difference = cv2.absdiff(color_preprocessed, bg_image)    
    binary_mask = posprocess(mask_difference)
    
    # visualize binary mask
    b = np.zeros(mask_difference.shape[:2], dtype = "uint8")
    r = np.zeros(mask_difference.shape[:2], dtype = "uint8")
    fgmask_diff_rgb = cv2.merge([b, mask_difference, r])
    b = np.zeros(binary_mask.shape[:2], dtype = "uint8")
    r = np.zeros(binary_mask.shape[:2], dtype = "uint8")
    binImg_rgb = cv2.merge([b, binary_mask, r])
    
    cnts = findContours(binary_mask)
    
    x, y, w, h = 0, 0, 0, 0

    for c in cnts:
        minArea = 6000
        if cv2.contourArea(c) < minArea:
            continue
            
        # in contour
        (x, y, w, h) = cv2.boundingRect(c)
        # cv2.rectangle(color_np, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # cv2.rectangle(fgmask_diff_rgb, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # cv2.rectangle(binImg_rgb, (x, y), (x + w, y + h), (0, 0, 255), 2)

    if not (x == 0 and y == 0 and w == 0 and h == 0):
        depth_np[0:y, :] = 0.
        depth_np[y+h:, :] = 0.
        depth_np[:, 0:x] = 0.
        depth_np[:, x+w:] = 0.

    # seg2!
    depth_np[binary_mask == 0] = 0.

    # ------------------------------------------------------------------------------------------------------------------------ #

    color_image = o3d.geometry.Image(color_np)
    depth_image = o3d.geometry.Image(depth_np)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image,
        convert_rgb_to_intensity=False) # false = colored   
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, get_intrinsic_matrix(None, imwidth, imheight))

    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    # o3d.visualization.draw_geometries([pcd])

    output_point_cloud(folder, pcd)

    cv2.rectangle(color_np, (x, y), (x + w, y + h), (255, 0, 0), 2)
    color_np = cv2.cvtColor(color_np, cv2.COLOR_BGR2RGB)
    # save rect image
    output_rect_image(folder, color_np)

    # visual binary mask
    combined = np.hstack((color_np, fgmask_diff_rgb, binImg_rgb))
    # cv2.imwrite('contour_img.png', color_np)
    # cv2.imwrite('subtraction_img.png', fgmask_diff_rgb)
    # cv2.imwrite('amplified_img.png', binImg_rgb)
    cv2.imshow("Combined", imutils.resize(combined, width=displayResize))

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="folder name")

    args = parser.parse_args()

    main(args.folder)