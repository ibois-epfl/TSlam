

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/imgproc.hpp>
#include "dirreader.h"
#include <vector>
#include <fstream>
#include <iostream>
using namespace std;

vector<string > readFileDir(string path){
    ifstream file(path);
    if(!file)throw std::runtime_error("Could not open "+path);

    vector<string >  result;
    while(!file.eof() ){
        string line;
        std::getline(file,line);
        if(line.empty())continue;
        std::replace(line.begin(),line.end(),',',' ');
        vector<string> elems(8);
        stringstream sstr;sstr<<line;
        for(int i=0;i<8;i++)sstr>>elems[i];

         if (  elems[1]!="nan" ){
             elems[2]=std::to_string( - std::stof(elems[2]));
              stringstream res;
             for(int i=0;i<8;i++) res<<elems[i]<<" ";
            result.push_back(res.str());
         }
    }
    return result;

}
void savePcd(vector<cv::Point3f> &points,string filepath){
std::ofstream filePCD (filepath, std::ios::binary );
filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z  \nSIZE 4 4 4  \nTYPE F F F \nCOUNT 1 1 1  \nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";

filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));

}
cv::Mat  getDisparityMap(cv::Mat left, cv::Mat right) {
    cv::Mat Depth, normalizedDepth;
    int ndisparities = 16 * 5;

//    auto sbm= cv::StereoSGBM::create( -80, 16*5, 3);
//       sbm->setPreFilterCap(  25);

//        sbm->setUniquenessRatio( 15);
//        sbm->setSpeckleWindowSize( 150);
//        sbm->setSpeckleRange (20);




    auto sbm=cv::StereoBM::create(ndisparities,5);
    sbm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
    sbm->setPreFilterCap(  25);
    sbm->setMinDisparity (-80);
    sbm->setUniquenessRatio( 15);
    sbm->setSpeckleWindowSize( 100);
    sbm->setSpeckleRange (20);

    sbm ->compute(left, right, Depth);

    double fx=100,fy=100;
    double bl=10;
    double cx=left.cols/2;
    double cy=left.rows/2;

    vector<cv::Point3f> points;
    for(int r=0;r<Depth.rows;r++){
           auto ptr=Depth.ptr<short>(r);
            for(int c=0;c<Depth.cols;c++)
                if(ptr[c]!=-1296 && abs(ptr[c])>200){
                    std::cout<<ptr[c]<<" " ;
                    double z= fx*bl / double(ptr[c]);
                    double x= (c- cx)*z / fx;
                    double y= (r- cy)*z / fy;
                    points.push_back(cv::Point3f(x,y,z));
                }
        }

    savePcd(points,"depth.pcd");

    normalize(Depth, normalizedDepth, 0, 255, CV_MINMAX, CV_8U);
    return normalizedDepth;
}
cv::Mat image,imageGRAY,auxIm;

void onmouse(int event, int x, int y, int flags, void *userdata){

     image.copyTo(auxIm);
    cv::line(auxIm,cv::Point2f(0,y),cv::Point2f(auxIm.cols,y),cv::Scalar(0,255,0),1);
    cv::imshow("image",auxIm);
    cv::waitKey(2);
}
int main(int argc,char **argv){


    try{
        if (argc!=2)throw std::runtime_error(" Usage: inDir ");
        cv::VideoCapture video;
        video.open(argv[1]);
        cv::namedWindow("image");
        cv::setMouseCallback("image",onmouse);

         char k=0;
        while(video.read(image) &&k!=27){
            cv::cvtColor(image,imageGRAY,CV_BGR2GRAY);


            cv::Mat disp=getDisparityMap(imageGRAY.colRange(0,imageGRAY.cols/2),imageGRAY.colRange(imageGRAY.cols/2,imageGRAY.cols));

            disp+=60;
            cv::imshow("disp",disp);
            cv::imshow("image",imageGRAY);
            k=cv::waitKey(0);

         }
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}

//int main(int argc,char **argv){


//    try{
//        if (argc!=2)throw std::runtime_error(" Usage: inDir ");
//        cv::VideoCapture video;
//        video.open(argv[1]);
//        cv::Mat image,imageGRAY;
//        cv::Size boardSize(7,5);
//cv::Size wSearchSize=cv::Size(5,5);
//        int counter=0;
//        bool process=false;
//        char k=0;
//        while(video.read(image) &&k!=27){
//            cv::cvtColor(image,imageGRAY,CV_BGR2GRAY);
//            cv::Mat left=imageGRAY.colRange(0,imageGRAY.cols/2),right=imageGRAY.colRange(imageGRAY.cols/2,imageGRAY.cols);
//            cv::Mat leftcopy=left.clone(),rightcopy=right.clone();

//            if(process){

//            vector <cv::Point2f> corners[2];
//            auto foundl = cv::findChessboardCorners(left, boardSize, corners[0],
//                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

//            if(foundl){
//                cornerSubPix(left, corners[0],wSearchSize , cv::Size(-1,-1),
//                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
//                                          5, 0.01));
//                drawChessboardCorners(left, boardSize, corners[0], foundl);
//            }

//            auto foundr = cv::findChessboardCorners(right, boardSize, corners[1],
//                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
//            if(foundr){
//                cornerSubPix(right, corners[1], wSearchSize, cv::Size(-1,-1),
//                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
//                                          5, 0.01));
//             drawChessboardCorners(right, boardSize, corners[1], foundr);
//            }

//}
//            cv::imshow("image",imageGRAY);



//            //            cv::Mat disp=getDisparityMap(imageGRAY.colRange(0,imageGRAY.cols/2),imageGRAY.colRange(imageGRAY.cols/2,imageGRAY.cols));

//  //          disp+=60;
//   //         cv::imshow("disp",disp);
//            k=cv::waitKey(0);
//            if(k=='s'){
//                string name=std::to_string(counter);
//                while(name.size()!=3)name="0"+name;
//                cv::imwrite(name+"-left.jpg",leftcopy);
//                cv::imwrite(name+"-right.jpg",rightcopy);
//                counter++;
//            }
//            if(k=='p')process=!process;
//        }
//    }catch(std::exception &ex){
//        cerr<<ex.what()<<endl;
//        return -1;
//    }
//}
