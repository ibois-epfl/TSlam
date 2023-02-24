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
        std::ofstream out(filepath);
        out.precision(17);
        CGAL::IO::write_PLY(out, this->m_GeometricSolver.getMeshOut());
        out.close();
    }
}