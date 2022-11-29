#pragma once

#include "ts_timber.hh"

namespace tslam
{
    class TSGSolver
    {
    public:
        TSGSolver();
        ~TSGSolver() = default;


        void reconstruct(std::shared_ptr<tslam::TSTimber>& timber);

        // void setTimber(std::shared_ptr<tslam::TSTimber> timber) {m_timber = timber; };

    // private:
        // TODO: place here auxiliary functions to reconstruct
        // void foo();

    // private:
    //     std::shared_ptr<tslam::TSTimber> m_timber;


    };
}