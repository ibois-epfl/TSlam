#include <iostream>
#include <opencv2/core.hpp>
#include <unordered_map>
#include "ucoslam.h"

class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };

namespace ucoslam {
struct DebugTest{
    static void removeMapPointObservation(ucoslam::Map &map,uint32_t point_idx,uint32_t frame_idx){
        map.removeMapPointObservation(point_idx,frame_idx,3);
    }
};
}
int main(int argc,char **argv){

    if(argc<3)throw std::runtime_error("Usage: inmap outmap");

    int minMarkerObservedTime = 5;
    int minNumProjPoints = 3;

    ucoslam::Map theMap;
    cout<<"reading map"<<endl;
    theMap.readFromFile(argv[1]);
    cout<<"Done"<<endl;
    
    // remove invalid markers
    set<uint32_t> validMarkers;
    set<uint32_t> invalidMarkers;

    for(auto marker: theMap.map_markers){
        if(!marker.second.pose_g2m.isValid()){
            invalidMarkers.insert(marker.first);
        } else {
            validMarkers.insert(marker.first);
        }
    }
    
    // remove invalid markers and redundant key frames
    set<uint32_t> deletedFrameIdx;
    set<uint32_t> unneededFrameIdx;
    unordered_map<uint32_t, int> markerObservedCounter; // marker.id -> total observed time

    // sort frame based on tag amount
    vector<pair<uint32_t, int> > frameTagAmount;
    for(auto kfIter = theMap.keyframes.begin(); kfIter != theMap.keyframes.end(); ++kfIter) {
        frameTagAmount.push_back({kfIter->idx, kfIter->markers.size()});
    }
    std::sort(frameTagAmount.begin(), frameTagAmount.end(), [](pair<unsigned int, int> left, pair<unsigned int, int> right) {
        return left.second > right.second;
    });
    for(auto frameIdx: frameTagAmount) {
        cout << frameIdx.first << ": ";
        for(auto marker: theMap.keyframes[frameIdx.first].markers) {
            cout << marker.id << " ";
        }
        cout << endl;
    }

    // go through every frame
    for(auto frameIdx: frameTagAmount){
        auto kf = theMap.keyframes[frameIdx.first];
        // check if this frame observed a marker that is not in the map yet
        bool isNeeded = false;
        unordered_map<uint32_t, int> tmpMarkerObservedCounter = markerObservedCounter;

        for(auto marker: kf.markers){
            if(invalidMarkers.find(marker.id) != invalidMarkers.end()){
                continue;
            }

            if(markerObservedCounter.find(marker.id) == markerObservedCounter.end()){
                isNeeded = true;
                tmpMarkerObservedCounter[marker.id] = 1;
            } else if (tmpMarkerObservedCounter[marker.id] <= minMarkerObservedTime){ // minial observed time
                isNeeded = true;
                tmpMarkerObservedCounter[marker.id] += 1;
            } else {
                tmpMarkerObservedCounter[marker.id] += 1;
            }
        }

        if(isNeeded){
            markerObservedCounter = tmpMarkerObservedCounter;
            // currentFrameInMap[kf.idx] = kf; // 
        } else {
            unneededFrameIdx.insert(kf.idx);
        }
    }
    
    theMap.removeKeyFrames(unneededFrameIdx, minNumProjPoints);
    
    deletedFrameIdx = unneededFrameIdx;
    unneededFrameIdx.clear();

    std::reverse(frameTagAmount.begin(), frameTagAmount.end());
    for(auto frameIdx: frameTagAmount) {
        if(deletedFrameIdx.find(frameIdx.first) != deletedFrameIdx.end()) continue;
        cout << frameIdx.first << ": ";
        for(auto marker: theMap.keyframes[frameIdx.first].markers) {
            cout << marker.id << " ";
        }
        cout << endl;
    }

    for(auto frameIdx: frameTagAmount){
        if(deletedFrameIdx.find(frameIdx.first) != deletedFrameIdx.end()) continue; // frame is already deleted
        auto kf = theMap.keyframes[frameIdx.first];
        cout << "Frame #" << frameIdx.first << ": ";
        bool isNeeded = false;
        unordered_map<uint32_t, int> tmpMarkerObservedCounter = markerObservedCounter;
        for(auto marker: kf.markers){
            cout << marker.id << ":" << tmpMarkerObservedCounter[marker.id] << " ";
            if(tmpMarkerObservedCounter[marker.id] <= minMarkerObservedTime){
                isNeeded = true;
                break;
            } else {
                tmpMarkerObservedCounter[marker.id] -= 1;
            }
        }
        if(!isNeeded){
            cout << "[Drop]";
            markerObservedCounter = tmpMarkerObservedCounter;
            unneededFrameIdx.insert(kf.idx);
        }
        cout << endl;
    }

    for(auto idx: unneededFrameIdx)
        deletedFrameIdx.insert(idx);

    theMap.removeKeyFrames(unneededFrameIdx, minNumProjPoints);

    // some point reference is not deleted? TODO: check the removeKeyFrame function
    for(auto &mapPoint: theMap.map_points){
        set<uint32_t> idxToErase;
        for(auto frame: mapPoint.frames){
            uint32_t frameIdx = frame.first;
            if(deletedFrameIdx.find(frameIdx) != deletedFrameIdx.end()){
                idxToErase.insert(frameIdx);
            }
        }
        for(auto idx: idxToErase){
            mapPoint.frames.erase(idx);
        }
    }

    theMap.removeUnUsedKeyPoints();

    theMap.saveToFile(argv[2]);

}

// gdb --args ./ucoslam_monocular '/home/tpp/Downloads/long-beam-1-480p-2.mp4' '/home/tpp/UCOSlam-IBOIS/test_result/calibration_pixel_480p.yml' "-voc" '/home/tpp/UCOSlam-IBOIS/orb.fbow' "-out" "test.map" '-map' '/home/tpp/UCOSlam-IBOIS/build/utils/long1-px-480p-combine-compressed.map'
// /home/tpp/Downloads/long-beam-1-480p-2.mp4 /home/tpp/UCOSlam-IBOIS/test_result/calibration_pixel_480p.yml -voc /home/tpp/UCOSlam-IBOIS/orb.fbow -out test.map -map /home/tpp/UCOSlam-IBOIS/build/utils/long1-px-480p-combine-compressed.map

// live /home/tpp/UCOSlam-IBOIS/test_result/calibration_webcam_new.yml -voc /home/tpp/UCOSlam-IBOIS/orb.fbow -out test_1 -map /home/tpp/UCOSlam-IBOIS/build/utils/sticker_6_oneshot.map -isInstancing