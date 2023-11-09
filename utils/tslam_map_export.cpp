/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

The modified version: Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)
The original project: Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas, MODIFIED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

//program that reads a map and exports it to ply or pcd
#include <iostream>
#include "map.h"


int main( int  argc , char**  argv )
{
    try {
        if (argc<2) throw  std::runtime_error ("Usage: map out.(pcd|ply|yml)");
        tslam::Map map;
        cerr<<"Reading map"<<endl;
        map.readFromFile(argv[1]);

        // map.exportToFile("test.pcd");
        // map.exportToFile("test.ply");
        // map.saveToMarkerMap("test.yml");

        cerr<<"Exporting map"<<endl;
        if( std::string(argv[2]).find(".yml")==std::string::npos)
            map.exportToFile(argv[2],cv::Scalar(125,125,125),cv::Scalar(255,0,0),cv::Scalar(0,0,255),{1111,1195,1129,1196,1141},cv::Scalar(0,255,0));
        else
            map.saveToMarkerMap(argv[2]);
        cerr<<"Saved to "<<argv[2]<<endl;

    } catch (std::exception &ex) {
        cerr<<ex.what()<<endl;
    }{}
}
