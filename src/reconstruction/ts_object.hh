#pragma once

namespace tslam
{
    /// TSObject is a simple interface parent class.
    class TSObject
    {
    public:
        TSObject () = default;
        ~TSObject () = default;

    protected:
        virtual void compute() = 0;
    };
}