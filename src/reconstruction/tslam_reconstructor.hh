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
        TSLAMReconstructor() = default;
        ~TSLAMReconstructor() = default;

        void loadMap(const std::string& filepath);

        void setParams();

        void run();

        void saveMesh(const std::string& filepath);

    private:
        TSTimber m_Timber;
        TSGeometricSolver m_GeometricSolver;
    };
}

#endif // __TSLAM_RECONSTRUCTOR_H__