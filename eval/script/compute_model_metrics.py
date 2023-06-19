import os
import numpy as np
import copy
from tqdm import tqdm
import matplotlib
import matplotlib.pyplot as plt
import csv
import visuals
import open3d as o3d

def export_combined_histogram(err_distance_data, save_path):
    tag_type = ["Stripe, low density", "Stripe, medium density", "Ring, low density", "Ring, medium density"]

    combined_histogram_save_path = os.path.join(save_path, "combined_histogram")
    os.makedirs(combined_histogram_save_path, exist_ok=True)

    for i in range(0, 4):
        batch_data = []
        for idx in range(0, 5):
            batch_data.append(err_distance_data[idx*4+i])
        visuals.draw_combined_model_metric_histogram(batch_data, 100, tag_type[i], combined_histogram_save_path, x_range = [0, 0.015], to_show = False)

def compute_model_metrics(root_path, save_path):
    err_distance_data = []
    for _id in tqdm(range(1, 21)):
        # Read the point cloud
        id = f"{_id:02d}"
        gt_pcd = os.path.join(root_path, f"{id}/{id}_pcd/{id}_aligned.ply")
        rec_pcd = os.path.join(root_path, f"{id}/{id}_pcd/{id}_reconstruct_aligned.ply")
        # gt_pcd = os.path.join(root_path, f"{id}_aligned.ply")
        # rec_pcd = os.path.join(root_path, f"{id}_reconstruct_aligned.ply")

        # First check if the file exists
        if not os.path.isfile(gt_pcd):
            print(f"[Error] Point Cloud File {gt_pcd} does not exist")
            exit()
        if not os.path.isfile(rec_pcd):
            print(f"[Error] Point Cloud File {rec_pcd} does not exist")
            exit()

        # print("Loading point cloud...")
        gt_pcd = o3d.io.read_point_cloud(gt_pcd)
        rec_pcd = o3d.io.read_point_cloud(rec_pcd)

        rec_pcd = rec_pcd.voxel_down_sample(voxel_size=0.005)

        dist = rec_pcd.compute_point_cloud_distance(gt_pcd)
        dist = np.asarray(dist)

        err_distance_data.append(dist)

        # export to csv
        csv_save_path = os.path.join(save_path, "err_distance_csv")
        os.makedirs(csv_save_path, exist_ok=True)
        with open(os.path.join(csv_save_path, f"{id}_err.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Distance"])
            for d in dist:
                writer.writerow([d])

        # metrics
        err_color_map = plt.get_cmap('viridis')
        histogram_save_path = os.path.join(save_path, "histogram")
        os.makedirs(histogram_save_path, exist_ok=True)
        visuals.draw_model_metric_histogram(dist, 50, id, histogram_save_path, range = (0, dist.max()))

        # color gt_pcd based on distance'viridis')
        dist = dist / 0.01
        pts_color = np.asarray(rec_pcd.colors)
        for i in range(len(pts_color)):
            pts_color[i] = err_color_map(dist[i])[:3]
        rec_pcd.colors = o3d.utility.Vector3dVector(pts_color)

        # move the point cloud center to origin
        rec_pcd.translate(-rec_pcd.get_center())

        # replace gt with rec or the open3d won't show the color
        gt_pcd.points = rec_pcd.points
        gt_pcd.colors = rec_pcd.colors

        # save the result
        render_save_path = os.path.join(save_path, "err_diff_render_result")
        os.makedirs(render_save_path, exist_ok=True)
        visuals.draw_model_comp_result(id, gt_pcd, render_save_path)
        
        # export the result point cloud
        ply_save_path = os.path.join(save_path, "err_diff_ply")
        os.makedirs(ply_save_path, exist_ok=True)
        save_path_ply = os.path.join(ply_save_path, f"{id}_reconstruct_aligned_err.ply")
        o3d.io.write_point_cloud(save_path_ply, rec_pcd)
    
    export_combined_histogram(err_distance_data, save_path)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Compute the metrics of the reconstructed point cloud')
    parser.add_argument('--data_path', type=str, help='The root path of the database', required=True)
    parser.add_argument('--out_path', type=str, help='The save path of the metrics', required=True)
    args = parser.parse_args()

    compute_model_metrics(args.data_path, args.out_path)