# VR2Gather Point Cloud Segmentation

This project aims to enhance the quality of 3D point cloud streaming in immersive virtual reality (VR) communication applications like VR2Gather. It addresses the limitation of capturing entire scenes as bounding boxes, including unwanted background objects along with the human participant.

## Problem Statement

The current implementation of VR2Gather captures scenes in a bounding box fashion, which includes not only the human participant but also other objects like chairs, cabinets, and various items within the bounding box. This results in noisy point clouds with many outliers that adversely affect the quality of the generated 3D representation.

## Solution Approach

To address this limitation, we propose applying a point cloud segmentation algorithm to automatically remove background outliers. This approach focuses on retaining only the foreground avatar point clouds, thereby enhancing the visual quality of the captured scene.

The solution involves processing 2D color and depth images captured by an Intel RealSense D455 camera to generate segmented point clouds. The segmentation process includes the following steps:

1. **Background Subtraction**: Isolate the foreground object by subtracting a background image from the current frame.
2. **Signal Amplification**: Enhance the detected regions using morphological operations like dilation and erosion.
3. **Contour Finding**: Detect the boundary box around the foreground object within the binary mask.
4. **Point Cloud Generation**: Create a 3D representation using only points within the detected contours.

## Experiment and Results

The experiment was conducted on 25 action frames, including sitting, standing, and ground-level actions. Two segmentation algorithms were evaluated:

1. **Segmented 1**: Considers only the contour coordinates to segment the point cloud.
2. **Segmented 2**: Considers both the contour coordinates and the signal mask for segmentation.

Quantitative results showed that Segmented 2, which incorporates the signal mask, performed better across accuracy, precision, recall, F1 score, and Intersection-over-Union (IoU) metrics.

Qualitative results visually demonstrated the superior ability of the Segmented 2 algorithm to isolate the foreground avatar cleanly, effectively removing background clutter and providing a more accurate representation.

## Conclusion

This project successfully implemented point cloud foreground segmentation on a single frame point cloud, demonstrating the potential for improving the quality of 3D point cloud streaming in immersive VR applications. However, further experimentation is needed to evaluate and optimize the algorithm's time cost for real-time communication scenarios.

## Future Work

To integrate this segmentation approach into real-time communication applications, further research is required to:

- Evaluate and optimize the algorithm's time cost for real-time performance.
- Explore alternative methods for background subtraction that do not require a pre-existing background image, allowing for dynamic and changing environments.

By addressing these limitations, we can enhance the feasibility and usability of 3D point cloud streaming in providing immersive and natural meeting experiences.