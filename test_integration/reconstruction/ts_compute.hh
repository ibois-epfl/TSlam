#pragma once

namespace tslam::Reconstruction
{
    /// TSCompute is a simple interface parent class for internal computation of geometric properties.
    class TSCompute
    {
    public:
        TSCompute () = default;
        ~TSCompute () = default;

    protected:
        virtual void compute() = 0;
    };
}