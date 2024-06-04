import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def save_img(pcd, output_path, ctr=None):
    vis = o3d.visualization.Visualizer()
    # vis.create_window(visible=False)  # Set the window size explicitly
    vis.create_window(visible=True)  # Set the window size explicitly
    vis.add_geometry(pcd)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    if ctr is not None:
        vis.get_view_control().convert_from_pinhole_camera_parameters(ctr, True)

    
    ctr = vis.get_view_control().convert_to_pinhole_camera_parameters()
    print(ctr)

    ctr = vis.get_view_control()
    parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera_2024-06-01-17-22-26.json")
    ctr.convert_from_pinhole_camera_parameters(parameters, True)

    vis.run()
    # vis.close()
    vis.capture_screen_image(output_path)
    vis.destroy_window()

    return ctr

def main(folder):
    reference_ply = os.path.join(folder, 'pc.ply')
    pcd = o3d.io.read_point_cloud(reference_ply)
    output_path = os.path.join(folder, 'pc.png')
    ctr = save_img(pcd, output_path)

    # get all plys under folder
    plys = [f for f in os.listdir(folder) if f.endswith('.ply')]
    for ply in plys:
        if ply == 'pc.ply':
            continue
        pcd = o3d.io.read_point_cloud(os.path.join(folder, ply))
        output_path = os.path.join(folder, ply.replace('.ply', '.png'))
        save_img(pcd, output_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="folder name")

    args = parser.parse_args()

    main(args.folder)
