#include "tslam_reconstructor.hh"

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Surface_mesh/IO/PLY.h>

#include <iostream>
#include <fstream>

#ifndef __has_include
  static_assert(false, "__has_include not supported");
#else
#  if __cplusplus >= 201703L && __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif

namespace tslam::Reconstruction
{
    TSLAMReconstructor::TSLAMReconstructor()
    {
        this->m_GeometricSolver = TSGeometricSolver();
        this->m_GeometricSolver.setShowVisualizer(false);
    }
    TSLAMReconstructor::TSLAMReconstructor(float radiusSearch,
                                           double creaseAngleThreshold,
                                           int minClusterSize,
                                           double maxPlnDist,
                                           double maxPlnAngle,
                                           double aabbScaleFactor,
                                           double maxPolyDist,
                                           double eps)
    {
        this->m_GeometricSolver = TSGeometricSolver();
        this->m_GeometricSolver.setShowVisualizer(false);

        this->setParams(radiusSearch,
                        creaseAngleThreshold,
                        minClusterSize,
                        maxPlnDist,
                        maxPlnAngle,
                        aabbScaleFactor,
                        maxPolyDist,
                        eps);
    }

    void TSLAMReconstructor::setParams(float radiusSearch,
                                       double creaseAngleThreshold,
                                       int minClusterSize,
                                       double maxPlnDist,
                                       double maxPlnAngle,
                                       double aabbScaleFactor,
                                       double maxPolyDist,
                                       double eps)
    {
        this->m_GeometricSolver.setRadiusSearch(radiusSearch);
        this->m_GeometricSolver.setCreaseAngleThreshold(creaseAngleThreshold);
        this->m_GeometricSolver.setMinClusterSize(minClusterSize);
        this->m_GeometricSolver.setMaxPlnDist2Merge(maxPlnDist);
        this->m_GeometricSolver.setMaxPlnAngle2Merge(maxPlnAngle);
        this->m_GeometricSolver.setAABBScaleFactor(aabbScaleFactor);
        this->m_GeometricSolver.setMaxPolyTagDist(maxPolyDist);
        this->m_GeometricSolver.setEPS(eps);
    }

    bool TSLAMReconstructor::run()
    {
        if (!this->m_GeometricSolver.getTimber().hasTags())
            if (this->m_MapPath.empty())
                throw std::runtime_error("[ERROR] No tags are loaded in the timber object.");
            else
                this->loadMap(this->m_MapPath);
        
        this->m_GeometricSolver.reconstruct();

        return (this->m_GeometricSolver.hasMesh()) ? true : false;
    }

    void TSLAMReconstructor::loadMap(const std::string& filepath)
    {
        if (!fs::exists(filepath))
            throw std::runtime_error("[ERROR] The file does not exist.");

        this->m_MapPath = filepath;
        
        this->m_GeometricSolver.getTimber().setPlaneTagsFromYAML(filepath);
    }

    void TSLAMReconstructor::saveMeshAsPLY(const std::string& dir, const std::string& filename)
    {
        if (!this->m_GeometricSolver.hasMesh())
            throw std::runtime_error("[ERROR] No mesh is reconstructed.");
        
        std::string filepath = dir + "/" + filename + ".ply";
        TSLAMReconstructor::saveMeshAsPLY(filepath);
    }

    void TSLAMReconstructor::saveMeshAsPLY(const std::string& filepath)
    {
        if (!this->m_GeometricSolver.hasMesh())
            throw std::runtime_error("[ERROR] No mesh is reconstructed.");

        std::ofstream out(filepath);
        out.precision(17);
        CGAL::IO::write_PLY(out, this->m_GeometricSolver.getMeshOut());
        out.close();
    }

    void TSLAMReconstructor::saveTagsAsPly(const std::string& dir, const std::string& filename)
    {
        if (!this->m_GeometricSolver.getTimber().hasTags())
            throw std::runtime_error("[ERROR] No tags are loaded in the timber object.");
        
        std::string filepath = dir + "/" + filename + ".ply";

        std::vector<TSRTag> tags = this->m_GeometricSolver.getTimber().getPlaneTags();

        std::vector<Eigen::Vector3d> corners;
        for (auto it = tags.begin(); it != tags.end(); it++)
        {
            Eigen::Vector3d cornerA = it->getCornerA();
            Eigen::Vector3d cornerB = it->getCornerB();
            Eigen::Vector3d cornerC = it->getCornerC();
            Eigen::Vector3d cornerD = it->getCornerD();

            corners.push_back(cornerA);
            corners.push_back(cornerB);
            corners.push_back(cornerC);
            corners.push_back(cornerD);
        }

        std::ofstream out(filepath);
        out.precision(17);
        out << "ply" << std::endl;
        out << "format ascii 1.0" << std::endl;
        out << "element vertex " << corners.size() << std::endl;
        out << "property float x" << std::endl;
        out << "property float y" << std::endl;
        out << "property float z" << std::endl;
        out << "property uchar red" << std::endl;
        out << "property uchar green" << std::endl;
        out << "property uchar blue" << std::endl;
        out << "property uchar alpha" << std::endl;
        out << "element face 0" << std::endl;
        out << "property list uchar int vertex_indices" << std::endl;
        out << "end_header" << std::endl;
        for (auto it = corners.begin(); it != corners.end(); it++)
        {
            out << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " 0 0 0 255" << std::endl;
        }

        out.close();

        std::cout << "[INFO] Tags are saved as " << filepath << std::endl;
    }
}