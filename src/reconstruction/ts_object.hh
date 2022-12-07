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

        //TODO: we should add function like e.g. calculate center used by many classes
    };
}