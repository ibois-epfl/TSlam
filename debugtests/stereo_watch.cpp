#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "stereorectify.h"
#include "dirreader.h"
#include <map>
using namespace  std;
cv::Mat composedStereo(cv::Mat left ,cv::Mat &right){
    cv::Mat comp(left.rows,left.cols*2,left.type());
    auto aux=comp.colRange(0,left.cols);
    left.copyTo(aux);
    aux=comp.colRange(left.cols,2*left.cols);
    right.copyTo(aux);
    for(int i=0;i<comp.rows;i+=20)
        cv::line(comp,cv::Point(0,i),cv::Point(comp.cols,i),cv::Scalar(0,255,0),1);
    return comp;
}


vector< string  > ReadImages(string dirPath)
{
    auto parseName=[](const std::string &name){

        auto pos=name.rfind("left");
        int leftright=-1;

        if(pos!=std::string::npos)  {
            leftright=0;
            pos+=4;
        }
        else{
            pos=name.rfind("right");
            if(pos==std::string::npos) return -1;
            else {
                leftright=1;
                pos+=5;
            }
        }
        string number;
        while( pos<name.size() ){
            if (isdigit(name[pos]))
                number.push_back(name[pos++]);
            else break;
        };
        return  std::stoi(number);
    };

    DirReader Dir;
    auto files=Dir.read(dirPath,".jpg .ppm .png .pgm .bmp",DirReader::Params(true));

    //now, find left ritgh names
    std::map<int,std::pair<string,string> > id_leftRight;

    for(auto file:files){
            int number=parseName(file);
            if( number!=-1){
                if( file.find("left")!=std::string::npos)
                    id_leftRight[number].first=file;
                else if(file.find("right")!=std::string::npos)
                        id_leftRight[number].second=file;
                else throw  std::runtime_error("Error in left right name");
            }
    }
    //consider only pairs
    vector<string> result;
    for(auto lr:id_leftRight){
        if(!lr.second.first.empty() && !lr.second.second.empty() ){
            result.push_back(lr.second.first);
            result.push_back(lr.second.second);
        }
    }
    return result;
}

int main(int argc,char **argv){

    try {
        if(argc!=3)throw std::runtime_error("Usage: dir stereo.yml");
        reslam::StereoRectify Rect;
        Rect.readFromXMLFile(argv[2]);


        auto images=ReadImages(argv[1]);


        char key=0;

        cv::Mat left,right;
        int idx=0;
        for(size_t i=0;i<images.size();i+=2){
            left=cv::imread(images[i]);
            right=cv::imread(images[i+1]);
            Rect.rectify(left,right);
            cv::imshow("left-right",composedStereo(Rect.getLeft(),Rect.getRight()));
             if( cv::waitKey(0)==27) break;
        }
    } catch (std::exception &ex) {

        cout<<ex.what()<<endl;
        return  -1;
    }{}
}

//int main(int argc,char **argv){

//    try {
//        if(argc!=4)throw std::runtime_error("Usage: video1 video2 stereo.yml");
//        reslam::StereoRectify Rect;
//        Rect.readFromXMLFile(argv[3]);


//        cv::VideoCapture vcap[2];
//        vcap[0].open(argv[1]);
//        vcap[1].open(argv[2]);

//        char key=0;

//        cv::Mat left,right;
//        while(key!=27 && vcap[0].grab() && vcap[1].grab()){
//            vcap[0].retrieve(left);
//            vcap[1].retrieve(right);
//            Rect.rectify(left,right);


//            cv::imshow("left-right",composedStereo(Rect.getLeft(),Rect.getRight()));
//            key=cv::waitKey(0);
//        }
//    } catch (std::exception &ex) {

//        cout<<ex.what()<<endl;
//        return  -1;
//    }{}
//}
