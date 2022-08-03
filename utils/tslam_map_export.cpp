/**
* This file is part of  TSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* TSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* TSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with TSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
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
