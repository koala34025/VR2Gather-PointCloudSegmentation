from realsense_depth import DepthCamera
import cv2
import numpy as np
import datetime
import imutils

resolution_width, resolution_height = (640, 480)
displayResize = 900
bg_path = "./images/bg_color.png"

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

def main():
    cam = DepthCamera(resolution_width, resolution_height)

    try:
        bg_image = cv2.imread(bg_path)
        bg_image = preprocess(bg_image)
    except:
        print("Unable to load background image")
        return

    while True:
        ret , depth_raw_frame, color_raw_frame = cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
            break
        
        color_np = np.asanyarray(color_raw_frame.get_data())
        depth_np = np.asanyarray(depth_raw_frame.get_data())

        # ------------------------------------------------------------------------------------------------------------------------ #
        color_preprocessed = preprocess(color_np)

        mask_difference = cv2.absdiff(color_preprocessed, bg_image)
        blue_channel = np.zeros(mask_difference.shape[:2], dtype = "uint8")
        red_channel = np.zeros(mask_difference.shape[:2], dtype = "uint8")
        mask_difference_image = cv2.merge([blue_channel, mask_difference, red_channel])

        binary_mask = posprocess(mask_difference)
        blue_channel_binary = np.zeros(binary_mask.shape[:2], dtype = "uint8")
        red_channel_binary = np.zeros(binary_mask.shape[:2], dtype = "uint8")
        binary_image = cv2.merge([blue_channel_binary, binary_mask, red_channel_binary])
        
        cnts = findContours(binary_mask)
        
        for c in cnts:
            minArea = 6000
            if cv2.contourArea(c) < minArea:
                continue

            (x, y, w, h) = cv2.boundingRect(c)
            # cv2.rectangle(color_np, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.rectangle(mask_difference_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(binary_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # ------------------------------------------------------------------------------------------------------------------------ #

        combined = np.hstack((color_np, mask_difference_image, binary_image))
        cv2.imshow("Combined", imutils.resize(combined, width=displayResize))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            folder = "images/"
            # time = datetime.datetime.now().strftime("%m%d%H%M%S")
            # cv2.imwrite(folder + time + "_color" + ".png", color_np)
            # cv2.imwrite(folder + time + "_depth" + ".png", depth_np.astype(np.uint16))
            # cv2.imwrite(folder + time + "_viDep" + ".png", cv2.applyColorMap(cv2.convertScaleAbs(depth_np, alpha=0.03), cv2.COLORMAP_JET))
            print("Saved images")

            cv2.imwrite(folder + "bg" + "_color" + ".png", color_np)
            cv2.imwrite(folder + "bg" + "_depth" + ".png", depth_np.astype(np.uint16))
            cv2.imwrite(folder + "bg" + "_viDep" + ".png", cv2.applyColorMap(cv2.convertScaleAbs(depth_np, alpha=0.03), cv2.COLORMAP_JET))
            # update bg image
            bg_image = cv2.imread(bg_path)
            bg_image = preprocess(bg_image)

    cam.release()

if __name__ == "__main__":
    main()