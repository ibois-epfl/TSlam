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
#ifndef tslam_FrameMatcher_H
#define tslam_FrameMatcher_H

#include "map_types/frame.h"
namespace  tslam {

/**
 * Class to analyze matches between two frames.
 * Internally it decides which implemetation to use. If Kp Dictionary is employed, then
 * advantage is taken to speedup search. Otherwise, flann search is employed
 */
namespace _impl {
class FrameMatcher_impl;
}
class FrameMatcher{
    std::shared_ptr<_impl::FrameMatcher_impl> _impl;
public:
    enum Type: int{TYPE_AUTO=0,TYPE_FLANN=1,TYPE_BOW=2};

    enum Mode: int{MODE_ALL=0,MODE_ASSIGNED=1,MODE_UNASSIGNED=2};


    FrameMatcher(Type t=TYPE_AUTO);
    void setParams(const Frame &trainFrame,Mode mode=MODE_ALL,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1);



    //normal match.
    //maxSearch number of comparisons in the  approximated search
    //only searchs for best and  second best
    std::vector<cv::DMatch> match(const Frame &queryFrame, Mode mode );

    //matches considering epipolar constrains
    //F12 is the SE3 matrix moving points from train 2 query
    //nn : number nearest neighbor searched
    //maxSearch number of comparisons in the  approximated search
    std::vector<cv::DMatch> matchEpipolar(const Frame &queryFrame, Mode mode, const cv::Mat &FQ2T );

private:
    Type _type;

};

}
#endif

