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

#ifndef TSLAM_POSEOPT_H
#define TSLAM_POSEOPT_H
#include <opencv2/core/core.hpp>
#include "basictypes/se3.h"
#include "map_types/frame.h"
#include "map.h"
namespace tslam{



class PnPSolver{
public:
    //returns the number of inlier matches
    static int solvePnp( const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &pose_io ,const int64_t currentKeyFrame=-1);

    static  bool solvePnPRansac(const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &posef2g_io,int maxIters=50);

};

}
#endif
