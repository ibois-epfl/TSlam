#pragma once

#include "ts_timber.hh"

namespace tslam
{
    class TSGeometricSolver
    {
    public:
        TSGeometricSolver() = default;
        TSGeometricSolver(std::shared_ptr<TSTimber> timber,
                          const uint& planeScaleFactor)
        : m_Timber(timber), m_PlaneScaleFactor(planeScaleFactor)
        {};
        ~TSGeometricSolver() = default;

        void reconstruct();
    
        inline void setTimber(std::shared_ptr<TSTimber> timber){m_Timber = timber; check4PlaneTags();};
        inline void setPlaneScaleFactor(const uint& planeScaleFactor){m_PlaneScaleFactor = planeScaleFactor;};

        bool check4PlaneTags();

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

    private:
        void scaleUpPlaneTags();

    private:
#ifndef PROFILER
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_start;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_end;
        std::chrono::milliseconds m_time_elapsed;
#endif
        std::shared_ptr<tslam::TSTimber> m_Timber;
        uint m_PlaneScaleFactor;
    };
}