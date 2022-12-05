#pragma once

#include "ts_timber.hh"

namespace tslam
{
    class TSGSolver
    {
    public:
        TSGSolver(std::shared_ptr<TSTimber> timber)
        : m_timber(timber)
        {};
        ~TSGSolver() = default;


        void reconstruct(std::shared_ptr<tslam::TSTimber>& timber);

        // void setTimber(std::shared_ptr<tslam::TSTimber> timber) {m_timber = timber; };

#ifndef PROFILER
        inline void timeStart(){m_time_start = std::chrono::high_resolution_clock::now(); };
        inline void timeEnd()
        {
            m_time_end = std::chrono::high_resolution_clock::now();
            m_time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(m_time_end - m_time_start);
            std::cout << "[PROFILER] Time elapsed: " << m_time_elapsed.count() << " ms" << std::endl;
        };
#else
        inline void timeStart(){};
        inline void timeEnd(){};
#endif
//     private:
//         // here are the uitlity functions for the reconstruction
//         void scaleUpPlaneTags();

    private:
#ifndef PROFILER
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_start;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_end;
        std::chrono::milliseconds m_time_elapsed;
#endif
        std::shared_ptr<tslam::TSTimber> m_timber;
    
    // public:__always_inline  // TEST
    //     std::shared_ptr<tslam::TSTimber> getTimber() {return m_timber; };  // TEST
    //     void setTimber(std::shared_ptr<tslam::TSTimber> timber) {m_timber = timber; };  // TEST
    };

}