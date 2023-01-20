#pragma once

#include <CGAL/Cartesian.h>

typedef int                                             Number_type;
typedef CGAL::Cartesian<Number_type>                    Kernel;

namespace tslam
{
    /// TSTassellation is a simple interface parent class for tassellation of the TS.
    class TSTassellation
    {
    public:
        TSTassellation () = default;
        ~TSTassellation () = default;

    // protected:
    //     virtual void tassellate() = 0;
    
        static void tassellate()
        {


        }
        
    };
}