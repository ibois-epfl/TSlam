import enum
import numpy as np
import open3d as o3d
import yaml
import sympy as sp
import random

exported_tag_yml = '/home/tpp/UCOSlam-IBOIS/build/utils/long_piece_fullba.yml' 
threshold_norm = 0.5
threshold_dist = 100

marker_rescale_factor = 1.9


def rn():
    return random.random()

def is_coplane(p1, p2, threshold_norm=threshold_norm, threshold_dist=threshold_dist, check_dist=False):
    return np.linalg.norm(p1 - p2) < threshold_norm or np.linalg.norm(p1 + p2) < threshold_norm


class Cluster:
    def __init__(self, norm, avg_vert, plane, meshes, ids, color=None):
        self.avg_norm = norm
        self.avg_vertices = avg_vert # the center of all the vertices
        self.plane = plane
        self.meshes = meshes
        self.ids = ids

        if color:
            self.color = color
        else:
            self.color = np.array([rn(), rn(), rn()])

        self.bbox_vertices = []
        self.bbox_mesh = o3d.geometry.TriangleMesh()

        self.intersect_planes = []

clusters = []

with open(exported_tag_yml, "r") as stream:
    stream = stream.read().replace('d:', 'd: ').replace("%YAML:1.0\n---","")
    map_data = yaml.safe_load(stream)

    ids = []

    timber_mesh = o3d.geometry.TriangleMesh()
    meshes = []
    labels = []

    for marker in map_data["aruco_bc_markers"]:
        ids.append(marker["id"])
        corners = np.array(marker["corners"]) * marker_rescale_factor

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(corners)
        mesh.triangles = o3d.utility.Vector3iVector(np.array([[0,1,2],[0,2,3]]))
        mesh.compute_vertex_normals()
        meshes.append(mesh)
    
    for i, mesh in enumerate(meshes):
        norm = np.average(np.asarray(mesh.triangle_normals), axis=0)
        avg_vert = np.average(np.asarray(mesh.vertices), axis=0)

        label = o3d.visualization.gui.Label3D(str(ids[i]), avg_vert)
        labels.append(label)

        if len(clusters) == 0:
            clusters.append(
                Cluster(norm, avg_vert,
                        sp.Plane(sp.Point3D(tuple(avg_vert)), tuple(norm)),
                        [mesh], [ids[i]])
                )
        else:
            is_clustered = False
            for c in clusters:
                if np.linalg.norm(c.avg_norm - norm) < threshold_norm and \
                    c.plane.distance(sp.Point3D(tuple(avg_vert))) < threshold_dist:

                    is_clustered = True

                    # update average norm first because it depends on len(id)
                    avg_norm = (c.avg_norm * len(c.ids) + norm) / (len(c.ids) + 1)
                    avg_vert = (c.avg_vertices * len(c.ids) + avg_vert) / (len(c.ids) + 1)
                    c.avg_norm = avg_norm
                    c.avg_vertices = avg_vert
                    c.plane = sp.Plane(sp.Point3D(tuple(avg_vert)), tuple(avg_norm))
                    c.meshes.append(mesh)
                    c.ids.append(ids[i])
                    break

            if not is_clustered:
                clusters.append(
                    Cluster(norm, avg_vert,
                        sp.Plane(sp.Point3D(tuple(avg_vert)), tuple(norm)),
                        [mesh], [ids[i]])
                    )


    # print clustered result
    print("There are",len(clusters), "clusters")
    for cluster in clusters:
        if len(cluster.ids) < 2:
            clusters.remove(cluster)
            continue

        print(cluster.ids)


    # look for plane intersection
    for i in range(len(clusters)):
        c1 = clusters[i]
        for k in range(i+1, len(clusters)):
            c2 = clusters[k]

            # consider to be parallel -> no intersection
            if is_coplane(c1.avg_norm, c2.avg_norm): continue
            
            inter_line = c1.plane.intersection(c2.plane)
            
            if len(inter_line) == 1:
                inter_line = inter_line[0]
            else:
                # TODO: should raise ERROR here
                continue

            for t in range(k+1, len(clusters)):
                c3 = clusters[t]

                if is_coplane(c1.avg_norm, c3.avg_norm): continue
                if is_coplane(c2.avg_norm, c3.avg_norm): continue

                inter_point = inter_line.intersection(c3.plane)
                
                if len(inter_point) == 1:
                    inter_point = np.array(inter_point[0]).astype(np.float64)
                    c1.intersect_planes.append([i, k, t])
                    c2.intersect_planes.append([i, k, t])
                    c3.intersect_planes.append([i, k, t])

                    c1.bbox_vertices.append(inter_point)
                    c2.bbox_vertices.append(inter_point)
                    c3.bbox_vertices.append(inter_point)
                else:
                    # TODO: should raise ERROR here
                    continue
    
    # update cluster bbox mesh and print result
    for clu_id, cluster in enumerate(clusters):
        p0_neighbors = []
        for i, ips in enumerate(cluster.intersect_planes[1:]):
            counter = 0
            for p_id in ips:
                if p_id in cluster.intersect_planes[0]:
                    counter += 1
            if counter == 2:
                p0_neighbors.append(i + 1) # add one because we start from 1 ([1:])
        p0_diagonal = [x for x in range(1, 4) if x not in p0_neighbors][0]
        
        # p0------N  //N = p0_neighbors
        #  |      |   
        #  N------D   //D = p0_diagonal

        edges_length = []
        edges_length.append(np.linalg.norm(cluster.bbox_vertices[p0_neighbors[0]] - cluster.bbox_vertices[p0_diagonal]))
        edges_length.append(np.linalg.norm(cluster.bbox_vertices[p0_neighbors[1]] - cluster.bbox_vertices[p0_diagonal]))
        edges_length.append(np.linalg.norm(cluster.bbox_vertices[p0_neighbors[0]] - cluster.bbox_vertices[0]))
        edges_length.append(np.linalg.norm(cluster.bbox_vertices[p0_neighbors[1]] - cluster.bbox_vertices[0]))

        print("---")
        print("cluster", clu_id, ":", cluster.ids)
        print("length:")
        for e in edges_length:
            print(e)

        cluster.bbox_mesh.vertices = o3d.utility.Vector3dVector(cluster.bbox_vertices)
        cluster.bbox_mesh.triangles = o3d.utility.Vector3iVector(np.array([[0]+p0_neighbors, [p0_diagonal]+p0_neighbors]))
        cluster.bbox_mesh.paint_uniform_color(cluster.color)

        timber_mesh += cluster.bbox_mesh 
        meshes.append(cluster.bbox_mesh)

    o3d.io.write_triangle_mesh("exported.ply", timber_mesh)
    
    o3d.visualization.draw_geometries(meshes,
                                        mesh_show_back_face=True,
                                        point_show_normal=True,
                                        )
                                        # lookat=np.array([-5, 1, 14]).reshape((3, 1)))

        
            