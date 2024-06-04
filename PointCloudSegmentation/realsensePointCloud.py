import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
import open3d as o3d
from realsense_depth import DepthCamera
from utils import createPointCloudO3D
from utils import depth2PointCloud
from utils import create_point_cloud_file2
from utils import write_point_cloud
import imutils
import os

resolution_width, resolution_height = (640, 480)

clip_distance_max = 5.00 #remove from the depth image all values above a given value (meters).
                          # Disable by giving negative value (default)
displayResize = 900
minArea = 6000

def preprocess(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #img = cv2.GaussianBlur(img, (47, 47), 0)

    return img

def posprocess(img):
    (T, img) = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)
    #img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
    #    cv2.THRESH_BINARY,11,2)
    img = cv2.dilate(img, None, iterations=42)
    img = cv2.erode(img, None, iterations=42)
    img = cv2.dilate(img, None, iterations=8)

    return img

def findContours(img):
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return cnts

def get_intrinsic_matrix(frame, imwidth, imheight):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    # print(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    out = o3d.camera.PinholeCameraIntrinsic(imwidth,  imheight, intrinsics.fx,
                                            intrinsics.fy, intrinsics.ppx,
                                            intrinsics.ppy)
    return out

def main():

    bgFrame = None

    bgPath = "bg_image.png"

    assert os.path.exists(bgPath)
    frame = cv2.imread(bgPath)
    print("bg frame loaded")
    frame_preprocess = preprocess(frame)
    bgFrame = frame_preprocess

    Realsensed435Cam = DepthCamera(resolution_width, resolution_height)

    depth_scale = Realsensed435Cam.get_depth_scale()

    while True:

        ret , depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
        

        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())

        # ------------------------------------------------------------------------------------------------------------------------ #
        frame_preprocess = preprocess(color_frame)

        fgmask_diff = cv2.absdiff(frame_preprocess, bgFrame)
        boundcolor = (255,255,255)
        fontsize = 3.5
        locY = 120
        b = np.zeros(fgmask_diff.shape[:2], dtype = "uint8")
        r = np.zeros(fgmask_diff.shape[:2], dtype = "uint8")

        fgmask_diff_rgb = cv2.merge([b, fgmask_diff, r])
        binImg = posprocess(fgmask_diff)

        b = np.zeros(binImg.shape[:2], dtype = "uint8")
        r = np.zeros(binImg.shape[:2], dtype = "uint8")

        binImg_rgb = cv2.merge([b, binImg, r])

        cnts = findContours(binImg)

        x, y, w, h = 0, 0, 0, 0 #bounding box=x, y, x+w, y+h

        QttyOfContours = 0
        for c in cnts:
            #if a contour has small area, it'll be ignored
            if cv2.contourArea(c) < minArea:
                continue

            QttyOfContours = QttyOfContours+1    

            #draw an rectangle "around" the object
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(color_frame, (x, y), (x + w, y + h), (0,0,255), 2)
            cv2.rectangle(fgmask_diff_rgb, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(binImg_rgb, (x, y), (x + w, y + h), (0, 0, 255), 2)

        combined = np.hstack((color_frame, fgmask_diff_rgb, binImg_rgb))
        # cv2.imshow("Combined", imutils.resize(combined, width=displayResize))

        # ------------------------------------------------------------------------------------------------------------------------ #

        
        #o3D library for construct point clouds with rgbd image and camera matrix
        # pcd = createPointCloudO3D(color_raw_frame, depth_raw_frame)
        color_frame2 = color_raw_frame
        depth_frame = depth_raw_frame

        # color_np = np.asanyarray(color_frame.get_data())
        color_np = color_frame
        color_frame = color_frame2
        color_origin_np = np.asanyarray(color_frame.get_data())
        imwidth, imheight, channel = color_np.shape
        # set color image to 0 outside of x y w h 
        # color_np = color_np.astype(float)
        # color_np[0:y, :] = 0.
        # color_np[y+h:, :] = 0.
        # color_np[:, 0:x] = 0.
        # color_np[:, x+w:] = 0.
        # print(color_np)
        color_image = o3d.geometry.Image(color_np)
        
        depth_np = np.asanyarray(depth_frame.get_data())
        
        # set depth image to 0 outside of x y w h 
        # print(depth_np)
        # depth_np = depth_np.astype(float)
        # print(x, y, w, h)
        
        # if not (x == 0 and y == 0 and w == 0 and h == 0):
        #     depth_np[0:y, :] = 0.
        #     depth_np[y+h:, :] = 0.
        #     depth_np[:, 0:x] = 0.
        #     depth_np[:, x+w:] = 0.

        depth_image = o3d.geometry.Image(depth_np)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image,
            convert_rgb_to_intensity=False) # false = colored
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, get_intrinsic_matrix(color_frame, imwidth, imheight))
        
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        o3d.visualization.draw_geometries([pcd]) 
        # cv2.imshow("Combined", imutils.resize(combined, width=displayResize))

        #plt.show()
        
        #print("frame shape:", color_frame.shape)
        

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # cv2.imwrite("frame_color.png", color_frame)
            # plt.imsave("frame_depth.png", depth_frame)
            break
        elif key == ord('s'):
            #cv2.imwrite("frame_color.png", color_frame)
            #plt.imsave("frame_depth.png", depth_np)
            cv2.imwrite("frame_depth.png", depth_np.astype(np.uint16))
            print(depth_np)
            cv2.imwrite(bgPath, color_origin_np)
            print(color_origin_np.shape, depth_np.shape)
            
            frame = color_origin_np
            frame_preprocess = preprocess(frame)
            bgFrame = frame_preprocess
            print("new bg frame saved, update bg frame")
    
    Realsensed435Cam.release() # release rs pipeline


if __name__ == '__main__':
    main()
