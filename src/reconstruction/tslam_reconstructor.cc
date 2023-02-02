#include "tslam_reconstructor.hh"
#include <filesystem>
#include <open3d/Open3D.h>


namespace tslam::Reconstruction
{
    TSLAMReconstructor::TSLAMReconstructor()
    {
        this->m_GeometricSolver = TSGeometricSolver();
        this->m_GeometricSolver.setShowVisualizer(false);
    }
    TSLAMReconstructor::TSLAMReconstructor(double creaseAngleThreshold,
                                           double maxPlnDist,
                                           double maxPlnAngle,
                                           double aabbScaleFactor,
                                           double maxPolyDist,
                                           double eps)
    {
        this->m_GeometricSolver = TSGeometricSolver();
        this->m_GeometricSolver.setShowVisualizer(false);

        this->setParams(creaseAngleThreshold,
                        maxPlnDist,
                        maxPlnAngle,
                        aabbScaleFactor,
                        maxPolyDist,
                        eps);
    }

    void TSLAMReconstructor::setParams(double creaseAngleThreshold,
                                       double maxPlnDist,
                                       double maxPlnAngle,
                                       double aabbScaleFactor,
                                       double maxPolyDist,
                                       double eps)
    {
        this->m_GeometricSolver.setCreaseAngleThreshold(creaseAngleThreshold);
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
        if (!std::filesystem::exists(filepath))
            throw std::runtime_error("[ERROR] The file does not exist.");

        this->m_MapPath = filepath;
        
        this->m_GeometricSolver.getTimber().setPlaneTagsFromYAML(filepath);
    }

    void TSLAMReconstructor::saveMeshAsPLY(const std::string& dir, const std::string& filename)
    {
        if (!this->m_GeometricSolver.hasMesh())
            throw std::runtime_error("[ERROR] No mesh is reconstructed.");
        
        std::string filepath = dir + "/" + filename;
        open3d::io::WriteTriangleMesh(filepath, this->m_GeometricSolver.getMeshOut(), true);
    }

    void TSLAMReconstructor::saveMeshAsXAC(const std::string& dir, const std::string& filename)
    {
        if (!this->m_GeometricSolver.hasMesh())
            throw std::runtime_error("[ERROR] No mesh is reconstructed.");
        
        std::string filepath = dir + "/" + filename + ".xac";

        // TODO: implement the XAC file writer
    }
}