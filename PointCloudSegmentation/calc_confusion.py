import open3d as o3d
import numpy as np
import argparse
import os

def make_dict(original_pcd, reference_pcd):
    # Initialize the dictionary
    point_dict = {}

    # First pass: original point cloud
    for point in np.asarray(original_pcd.points):
        point = np.around(point, 6)
        point_dict[tuple(point)] = 'background'
    len_before = len(point_dict)

    # Second pass: reference point cloud
    for point in np.asarray(reference_pcd.points):
        point_dict[tuple(point)] = 'foreground'
    len_after = len(point_dict)

    # print("Before: {}, After: {}".format(len_before, len_after))

    return point_dict

def compute_confusion_matrix(point_dict, segmented_pcd):
    # Third pass: segmented point cloud
    for point in np.asarray(segmented_pcd.points):
        point = np.around(point, 6)
        if tuple(point) in point_dict:
            if point_dict[tuple(point)] == 'background':
                point_dict[tuple(point)] = 'FP'
            else:
                point_dict[tuple(point)] = 'TP'

    # Fourth pass: for those not false positive and true positive
    for point, label in point_dict.items():
        if label == 'background':
            point_dict[point] = 'TN'
        elif label == 'foreground':
            point_dict[point] = 'FN'

    # Compute confusion matrix
    confusion_matrix = {
        'TP': 0,
        'FP': 0,
        'TN': 0,
        'FN': 0
    }

    for label in point_dict.values():
        confusion_matrix[label] += 1

    return confusion_matrix

def main(folder):
    original_ply_path = os.path.join(folder, "pc.ply")
    segmented_ply_path = os.path.join(folder, "segmented.ply")
    segmented2_ply_path = os.path.join(folder, "segmented2.ply")
    reference_ply_path = os.path.join(folder, "reference.ply")

    original_pcd = o3d.io.read_point_cloud(original_ply_path)
    segmented_pcd = o3d.io.read_point_cloud(segmented_ply_path)
    segmented2_pcd = o3d.io.read_point_cloud(segmented2_ply_path)
    reference_pcd = o3d.io.read_point_cloud(reference_ply_path)

    # calculate confusion matrix
    point_dict = make_dict(original_pcd, reference_pcd)
    confusion_matrix_segmented = compute_confusion_matrix(point_dict, segmented_pcd)
    confusion_matrix_segmented2 = compute_confusion_matrix(point_dict, segmented2_pcd)

    with open(os.path.join(folder, "confusion.txt"), "w") as f:
        f.write("Segmented: {}\n".format(confusion_matrix_segmented))
        f.write("Segmented2: {}\n".format(confusion_matrix_segmented2))
        print("Segmented: {}, Segmented2: {}".format(confusion_matrix_segmented, confusion_matrix_segmented2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="folder name")

    args = parser.parse_args()

    main(args.folder)