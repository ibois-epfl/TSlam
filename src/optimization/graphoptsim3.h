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
#ifndef TSLAM_GRAPHOPTSIM3
#define TSLAM_GRAPHOPTSIM3
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include "map_types/covisgraph.h"


namespace tslam{




void    loopClosurePathOptimizationg2o(const std::vector<std::pair<uint32_t,uint32_t> > &edges, uint32_t IdClosesLoopNew, uint32_t IdClosesLoopOld, cv::Mat expectedPoseNew, std::map<uint32_t, cv::Mat> &optimPoses, bool bFixScale,std::map<uint64_t,float> edgeWeight={});


};

#endif
