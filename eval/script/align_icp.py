import os
import numpy as np
import open3d as o3d
import copy
from tqdm import tqdm

def draw_result(pc1, pc2):
    bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pc1.points)
    o3d.visualization.draw_geometries([pc1, pc2],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=bbox.get_center(),
                                      up=[-0.3402, -0.9189, -0.1996])

def rotate_to_align_axis(pcd: o3d.geometry.PointCloud):
    pcd = copy.deepcopy(pcd)
    pts = np.asarray(pcd.points)
    pts = pts.T

    means = np.mean(pts, axis=1)
    cov = np.cov(pts)
    eval, evec = np.linalg.eig(cov)

    centered_pts = pts - means[:,np.newaxis]

    aligned_pts = np.matmul(evec.T, centered_pts)

    pcd.points = o3d.utility.Vector3dVector(aligned_pts.T)

    return pcd

def align_pcd(source, target, threshold=20):
    print("Apply point-to-point ICP...")
    source = copy.deepcopy(source)
    target = copy.deepcopy(target)

    trans_init = np.eye(4)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=3000, relative_fitness=1e-6, relative_rmse=1e-6)
    )
    print(reg_p2p)
    transformed_source = source.transform(reg_p2p.transformation)
    
    return transformed_source, target, reg_p2p


if __name__ == "__main__":
    root_path = "/mnt/c/Users/eyang/Desktop/cleaned_pcd"
    
    for _id in tqdm(range(1, 21)):
        # Read the point cloud
        id = f"{_id:02d}"
        pcd_01 = os.path.join(root_path, f"{id}_01.ply")
        pcd_02 = os.path.join(root_path, f"{id}_02.ply")

        # First check if the file exists
        if not os.path.isfile(pcd_01):
            print(f"[Error] Point Cloud File {pcd_01} does not exist")
            exit()
        if not os.path.isfile(pcd_02):
            print(f"[Error] Point Cloud File {pcd_02} does not exist")
            exit()

        print("Loading point cloud:")
        print(pcd_01)
        print(pcd_02)
        pcd_01 = o3d.io.read_point_cloud(pcd_01)
        pcd_02 = o3d.io.read_point_cloud(pcd_02)
        
        # estimate the normal
        pcd_01.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pcd_02.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        trial = []
        print("\n----- First Trial -----")
        align_pcd_01, align_pcd_02, reg_p2p = align_pcd(pcd_01, pcd_02)
        # trial.append(
        #     {
        #         "pcd_01": align_pcd_01,
        #         "pcd_02": align_pcd_02,
        #         "reg_p2p": reg_p2p
        #     }
        # )
        o3d.io.write_point_cloud(os.path.join(root_path, f"{id}_aligned_F_01.ply"), align_pcd_01)
        o3d.io.write_point_cloud(os.path.join(root_path, f"{id}_aligned_F_02.ply"), align_pcd_02)
        draw_result(align_pcd_01, align_pcd_02)

        print("\n----- Second Trial -----")
        align_pcd_02, align_pcd_01, reg_p2p = align_pcd(align_pcd_02, align_pcd_01)
        # trial.append(
        #     {
        #         "pcd_01": align_pcd_01,
        #         "pcd_02": align_pcd_02,
        #         "reg_p2p": reg_p2p
        #     }
        # )
        o3d.io.write_point_cloud(os.path.join(root_path, f"{id}_aligned_S_01.ply"), align_pcd_01)
        o3d.io.write_point_cloud(os.path.join(root_path, f"{id}_aligned_S_02.ply"), align_pcd_02)
        draw_result(align_pcd_01, align_pcd_02)
        # print("\n----- Result -----")
        # if len(trial[0]["reg_p2p"].correspondence_set) > len(trial[1]["reg_p2p"].correspondence_set):
        #     best_trial = trial[0]
        #     print("Select the trial[0] as the best trial")
        # else:
        #     best_trial = trial[1]
        #     print("Select the trial[1] as the best trial")
        
        # export the result
        # print("Export the result...")
        # o3d.io.write_point_cloud(os.path.join(root_path, f"{id}_01_aligned.ply"), best_trial["pcd_01"])
        # o3d.io.write_point_cloud(os.path.join(root_path, f"{id}_02_aligned.ply"), best_trial["pcd_02"])

        # draw_result(best_trial["pcd_01"], best_trial["pcd_02"])