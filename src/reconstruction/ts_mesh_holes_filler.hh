#pragma once

#include "ts_geo_util.hh"  // FIXME: test

#include <open3d/Open3D.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

#include <boost/lexical_cast.hpp>

// FIXME: test polygon soup
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>

#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/IO/polygon_soup_io.h>


#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <tuple>
#include <vector>





// typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                     Point_3;
typedef CGAL::Surface_mesh<Point_3>                           Mesh_srf;

typedef boost::graph_traits<Mesh_srf>::vertex_descriptor        vertex_descriptor;
typedef boost::graph_traits<Mesh_srf>::halfedge_descriptor      halfedge_descriptor;
typedef boost::graph_traits<Mesh_srf>::face_descriptor          face_descriptor;



namespace PMP = CGAL::Polygon_mesh_processing;

namespace tslam::Reconstruction
{
    class TSMeshHolesFiller
    {
    public: __always_inline  ///< fill holes
        // FIXME: check if this function is needed
        static bool is_small_hole(halfedge_descriptor h, Mesh_srf & mesh,
                   double max_hole_diam, int max_num_hole_edges)
        {
            int num_hole_edges = 0;
            CGAL::Bbox_3 hole_bbox;
            for (halfedge_descriptor hc : CGAL::halfedges_around_face(h, mesh))
            {
                const Point_3& p = mesh.point(target(hc, mesh));
                hole_bbox += p.bbox();
                ++num_hole_edges;
                // Exit early, to avoid unnecessary traversal of large holes
                if (num_hole_edges > max_num_hole_edges) return false;
                if (hole_bbox.xmax() - hole_bbox.xmin() > max_hole_diam) return false;
                if (hole_bbox.ymax() - hole_bbox.ymin() > max_hole_diam) return false;
                if (hole_bbox.zmax() - hole_bbox.zmin() > max_hole_diam) return false;
            }
            return true;
        }

        /**
         * @brief It fills the holes in the mesh using the CGAL library. Normals are recomputed accordingly.
         * 
         * @param cgalMesh the input CGAL mesh
         */
        static void fillHoles(Mesh_srf &cgalMesh)
        {
            // // convert the mesh to a polygon soup
            // std::vector<Point_3> points;
            // std::vector<std::vector<std::size_t> > polygons;
            // PolygonRange range = CGAL::faces(cgalMesh);
            // CGAL::Polygon_mesh_processing::polygon_mesh_to_polygon_soup(points, polygons, cgalMesh);


            /////////////////////////////////////////////////////

            double max_hole_diam   = -1.0;
            int max_num_hole_edges = -1;
            unsigned int nb_holes = 0;
            std::vector<halfedge_descriptor> border_cycles;
            // collect one halfedge per boundary cycle
            PMP::extract_boundary_cycles(cgalMesh, std::back_inserter(border_cycles));
            for(halfedge_descriptor h : border_cycles)
            {
                // if(max_hole_diam > 0 && max_num_hole_edges > 0 &&
                // !is_small_hole(h, cgalMesh, max_hole_diam, max_num_hole_edges))
                // continue;
                std::vector<face_descriptor>  patch_facets;
                std::vector<vertex_descriptor> patch_vertices;
                bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(cgalMesh,
                                                                                h,
                                                                                std::back_inserter(patch_facets),
                                                                                std::back_inserter(patch_vertices)));
                // std::cout << "* Number of facets in constructed patch: " << patch_facets.size() << std::endl;
                // std::cout << "  Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
                // std::cout << "  Is fairing successful: " << success << std::endl;
                ++nb_holes;
            }
            std::cout << std::endl;
            // std::cout << nb_holes << " holes have been filled" << std::endl;
            // CGAL::IO::write_polygon_mesh("filled_SM.off", cgalMesh, CGAL::parameters::stream_precision(17));
            // std::cout << "Mesh written to: filled_SM.off" << std::endl;
        };


        static Mesh_srf cvtPolygon2CGALPolygonSoup(std::vector<TSPolygon>& polygons)
        {
            Mesh_srf meshOut;
            std::vector<Point_3> pointsRange;
            std::vector<std::vector<std::size_t> > polygonsRange;

            for (auto& polygon : polygons)
            {
                std::vector<std::size_t> polygonRange;
                for (auto& point : polygon.getVertices())
                {
                    pointsRange.push_back(Point_3(point[0], point[1], point[2]));
                    polygonRange.push_back(pointsRange.size() - 1);
                }
                polygonsRange.push_back(polygonRange);
            }

            // Visitor visitor;
            // CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons, CGAL::parameters::visitor(visitor));

            CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(pointsRange, polygonsRange, meshOut);

            return meshOut;
        };

        static bool cleanOutCGALMesh(Mesh_srf &cgalMesh)
        {
            std::vector<Point_3> points;
            std::vector<std::vector<std::size_t> > polygons;

            // convert the mesh to a polygon soup
            PMP::polygon_mesh_to_polygon_soup(cgalMesh, points, polygons);
            size_t nbrRmvPoints = 0;
            nbrRmvPoints = PMP::merge_duplicate_points_in_polygon_soup(points, polygons);
            std::cout << "Removed " << nbrRmvPoints << " duplicate points" << std::endl;

            std::cout << "Before reparation, the soup has " << points.size() << " vertices and " << polygons.size() << " faces" << std::endl;
            PMP::repair_polygon_soup(points, polygons);
            std::cout << "After reparation, the soup has " << points.size() << " vertices and " << polygons.size() << " faces" << std::endl;
            PMP::orient_polygon_soup(points, polygons);

            Mesh_srf meshOut;
            PMP::polygon_soup_to_polygon_mesh(points, polygons, meshOut);
            cgalMesh = meshOut;

            return true;
        }



    public: __always_inline  ///< converter mesh o3d/CGAL
        /**
         * @brief convert open3d mesh to CGAL mesh
         * 
         * @param mesh the input triangle open3d mesh
         * @param cgalMesh the output CGAL mesh
         * @return true if the conversion is successful
         */
        static bool cvtO3d2CGALMesh(const open3d::geometry::TriangleMesh &mesh,
                                    Mesh_srf &cgalMesh)
        {
            if (!mesh.HasVertices() || !mesh.HasTriangles())
                return false;
            cgalMesh.clear();

            // add a custom property to store normals
            Mesh_srf::Property_map<face_descriptor, Point_3> f_normal_map;
            bool created;
            boost::tie(f_normal_map, created) = cgalMesh.add_property_map<face_descriptor, Point_3>("f:normals", Point_3(0, 0, 0));
            assert(created);
            for (uint i = 0; i < mesh.triangles_.size(); ++i)
            {
                Eigen::Vector3d pt0 = mesh.vertices_[mesh.triangles_[i](0)];
                Eigen::Vector3d pt1 = mesh.vertices_[mesh.triangles_[i](1)];
                Eigen::Vector3d pt2 = mesh.vertices_[mesh.triangles_[i](2)];

                vertex_descriptor v1 = cgalMesh.add_vertex(Point_3(pt0(0), pt0(1), pt0(2)));
                vertex_descriptor v2 = cgalMesh.add_vertex(Point_3(pt1(0), pt1(1), pt1(2)));
                vertex_descriptor v3 = cgalMesh.add_vertex(Point_3(pt2(0), pt2(1), pt2(2)));

                face_descriptor f = cgalMesh.add_face(v1, v2, v3);
                f_normal_map[f] = Point_3(mesh.triangle_normals_[i](0), mesh.triangle_normals_[i](1), mesh.triangle_normals_[i](2));
            }

            return true;
        }
        /**
         * @brief convert CGAL mesh to open3d mesh
         * 
         * @param cgalMesh the input CGAL mesh
         * @param mesh the output open3d triangle mesh
         * @return true if the conversion is successful
         */
        static bool cvtCGAL2O3dMesh(const Mesh_srf &cgalMesh,
                                    open3d::geometry::TriangleMesh &mesh)
        {
            if (cgalMesh.is_empty())
                return false;
            mesh.Clear();

            // store vertices
            std::vector<Eigen::Vector3d> vertices;
            for (auto& v : cgalMesh.vertices())
            {
                const auto& p = cgalMesh.point(v);
                double x = CGAL::to_double(p.x());
                double y = CGAL::to_double(p.y());
                double z = CGAL::to_double(p.z());

                // if the vertex is not in the vector vertices, add it
                if (std::find(mesh.vertices_.begin(), mesh.vertices_.end(), Eigen::Vector3d(x, y, z)) == mesh.vertices_.end())
                    mesh.vertices_.push_back(Eigen::Vector3d(x, y, z));
            }

            // store faces
            for (auto& f : cgalMesh.faces())
            {
                std::vector<vertex_descriptor> vertices;
                const auto& hf = cgalMesh.halfedge(f);
                for (auto v : CGAL::vertices_around_face(hf, cgalMesh))
                    vertices.push_back(v);

                // finf vertices[0] in mesh.vertices_
                auto& pt_2_look_for_0 = cgalMesh.point(vertices[0]);
                auto it_0 = std::find_if(mesh.vertices_.begin(), mesh.vertices_.end(), [&](const Eigen::Vector3d& pt) {
                    return pt(0) == CGAL::to_double(pt_2_look_for_0.x()) &&
                           pt(1) == CGAL::to_double(pt_2_look_for_0.y()) &&
                           pt(2) == CGAL::to_double(pt_2_look_for_0.z());
                });
                if (it_0 == mesh.vertices_.end())
                    continue;
                int idx_0 = std::distance(mesh.vertices_.begin(), it_0);

                // finf vertices[1] in mesh.vertices_
                auto& pt_2_look_for_1 = cgalMesh.point(vertices[1]);
                auto it_1 = std::find_if(mesh.vertices_.begin(), mesh.vertices_.end(), [&](const Eigen::Vector3d& pt) {
                    return pt(0) == CGAL::to_double(pt_2_look_for_1.x()) &&
                           pt(1) == CGAL::to_double(pt_2_look_for_1.y()) &&
                           pt(2) == CGAL::to_double(pt_2_look_for_1.z());
                });
                if (it_1 == mesh.vertices_.end())
                    continue;
                int idx_1 = std::distance(mesh.vertices_.begin(), it_1);

                // finf vertices[2] in mesh.vertices_
                auto& pt_2_look_for_2 = cgalMesh.point(vertices[2]);
                auto it_2 = std::find_if(mesh.vertices_.begin(), mesh.vertices_.end(), [&](const Eigen::Vector3d& pt) {
                    return pt(0) == CGAL::to_double(pt_2_look_for_2.x()) &&
                           pt(1) == CGAL::to_double(pt_2_look_for_2.y()) &&
                           pt(2) == CGAL::to_double(pt_2_look_for_2.z());
                });
                if (it_2 == mesh.vertices_.end())
                    continue;
                int idx_2 = std::distance(mesh.vertices_.begin(), it_2);

                mesh.triangles_.push_back(Eigen::Vector3i(idx_0, idx_1, idx_2));
            }

            // store triangle normals
            Mesh_srf::Property_map<face_descriptor, Point_3> fnormals;
            fnormals = cgalMesh.property_map<face_descriptor, Point_3>("f:normals").first;
            for (auto& f : cgalMesh.faces())
            {
                Point_3 n = fnormals[f];
                double x = CGAL::to_double(n.x());
                double y = CGAL::to_double(n.y());
                double z = CGAL::to_double(n.z());
                mesh.triangle_normals_.push_back(Eigen::Vector3d(x, y, z));
            }

            return true;
        };
    };
}