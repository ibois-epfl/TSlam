#ifndef __TSLAM_RECONSTRUCTOR_H__
#define __TSLAM_RECONSTRUCTOR_H__

#include "tslam_reconstructor_exports.hh"

#include "ts_rtag.hh"
#include "ts_rtstripe.hh"
#include "ts_timber.hh"
#include "ts_geo_util.hh"
#include "ts_tassellation.hh"
#include "ts_geometric_solver.hh"

// TODO: extenralize I/O saving from geometric solver

namespace tslam::Reconstruction
{
    // build an API
    class TSLAM_RECONSTRUCTOR_API TSLAMReconstructor
    {
    public:
        TSLAMReconstructor();
        ~TSLAMReconstructor() = default;

    public:  ///< main funcs
        /**
         * @brief Set the parameters for the geometric solver.
         * 
         * @param creaseAngleThreshold The threshold for detection of crease's angle (the smaller the more creases will be detected)
         * @param maxPlnDist The scale factor for scaleing up the AABB of the timber element
         * @param maxPlnAngle The maximal distance between a polygon and a tag to be considered as a candidate face
         * @param aabbScaleFactor The maximal distance between planes of stripes to be eligible for merging
         * @param maxPolyDist The maximal angle difference in degs between two planes'normals' angle to be eligible for merging
         * @param eps The tolerance for all the computation (e.g. for the intersections)
         */
        void setParams(double creaseAngleThreshold,
                       double maxPlnDist,
                       double maxPlnAngle,
                       double aabbScaleFactor,
                       double maxPolyDist,
                       double eps);

        /**
         * @brief Run the reconstruction process.
         * 
         * @return true if the mesh is reconstructed successfully.
         * @return false if the mesh is not reconstructed.
         */
        bool run();

    public:  ///< I/O
        /**
         * @brief Load the plane tags from the yaml file attached to the timber element map.
         * 
         * @param filepath path of the map yaml file
         */
        void loadMap(const std::string& filepath);
        /**
         * @brief Save the reconstructed mesh as a PLY file.
         * 
         * @param dir directory to save the mesh
         * @param filename name of the mesh file
         */
        void saveMeshAsPLY(const std::string& dir, const std::string& filename);
        /**
         * @brief Save the reconstructed mesh as a XAC file.
         * 
         * @see saveMeshAsPLY
         */
        // TODO: implement the XAC file writer
        void saveMeshAsXAC(const std::string& dir, const std::string& filename);
    
    public:  ///< special getters
        /**
         * @brief Get the timber object.
         * 
         * @return TSTimber& timber object
         */
        TSTimber& getTimber() { return this->m_Timber; }
        /**
         * @brief Get the geometric solver object.
         * 
         * @return TSGeometricSolver& geometric solver object
         */
        TSGeometricSolver& getGeometricSolver() { return this->m_GeometricSolver; }

    private:  ///< internal members
        // const std::string m_YMLpath;
        TSTimber m_Timber;
        TSGeometricSolver m_GeometricSolver;
    };
}

#endif // __TSLAM_RECONSTRUCTOR_H__