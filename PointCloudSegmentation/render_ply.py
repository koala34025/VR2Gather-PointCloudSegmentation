import open3d as o3d

def render_ply(ply_path):
    pcd = o3d.io.read_point_cloud(ply_path)
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    ply_path = r"output\bg2\ground1\pc.ply"
    render_ply(ply_path)