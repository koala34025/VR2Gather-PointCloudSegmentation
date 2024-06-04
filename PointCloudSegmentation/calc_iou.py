import open3d as o3d
import numpy as np
import argparse
import os

def compute_iou(segmented_pcd, groundtruth_pcd):
    # calculate iou
    segmented_box = segmented_pcd.get_axis_aligned_bounding_box()
    groundtruth_box = groundtruth_pcd.get_axis_aligned_bounding_box()

    segmented_max_bound = segmented_box.get_max_bound()
    segmented_min_bound = segmented_box.get_min_bound()
    groundtruth_max_bound = groundtruth_box.get_max_bound()
    groundtruth_min_bound = groundtruth_box.get_min_bound()

    # calculate intersection
    intersection = 1
    for i in range(3):
        if segmented_max_bound[i] < groundtruth_min_bound[i] or groundtruth_max_bound[i] < segmented_min_bound[i]:
            intersection = 0
            break
        intersection *= min(segmented_max_bound[i], groundtruth_max_bound[i]) - max(segmented_min_bound[i], groundtruth_min_bound[i])

    # calculate union
    segmented_volume = 1
    groundtruth_volume = 1
    for i in range(3):
        segmented_volume *= segmented_max_bound[i] - segmented_min_bound[i]
        groundtruth_volume *= groundtruth_max_bound[i] - groundtruth_min_bound[i]
    union = segmented_volume + groundtruth_volume - intersection
    
    if union == 0:
        iou = 0
    else:
        iou = intersection / union
    return iou

def main(folder):
    original_ply_path = os.path.join(folder, "pc.ply")
    segmented_ply_path = os.path.join(folder, "segmented.ply")
    segmented2_ply_path = os.path.join(folder, "segmented2.ply")
    reference_ply_path = os.path.join(folder, "reference.ply")

    original_pcd = o3d.io.read_point_cloud(original_ply_path)
    segmented_pcd = o3d.io.read_point_cloud(segmented_ply_path)
    segmented2_pcd = o3d.io.read_point_cloud(segmented2_ply_path)
    reference_pcd = o3d.io.read_point_cloud(reference_ply_path)

    # calculate iou and output txt file
    iou_original = compute_iou(original_pcd, reference_pcd)
    iou_segmented = compute_iou(segmented_pcd, reference_pcd)
    iou_segmented2 = compute_iou(segmented2_pcd, reference_pcd)

    with open(os.path.join(folder, "iou.txt"), "w") as f:
        f.write("Original: {}\n".format(iou_original))
        f.write("Segmented: {}\n".format(iou_segmented))
        f.write("Segmented2: {}\n".format(iou_segmented2))
        print("Original: {}, Segmented: {}, Segmented2: {}".format(iou_original, iou_segmented, iou_segmented2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="folder name")

    args = parser.parse_args()

    main(args.folder)