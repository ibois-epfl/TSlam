#pragma once

namespace tslam
{
    /// TSCompute is a simple interface parent class for internal computation of geometric properties.
    class TSCompute
    {
    public:
        TSCompute () = default;
        ~TSCompute () = default;

    protected:
        virtual void compute() = 0;

        //TODO: we should add function like e.g. calculate center used by many classes
    };
}