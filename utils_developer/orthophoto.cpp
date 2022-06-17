/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/

/**Reads a map and the video sequence it was created with, and generates the required data for PMVS 3D reconstruction software
 * https://github.com/pmoulon/CMVS-PMVS
 */
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "basictypes/cvversioning.h"
#include "map.h"
using namespace  std;
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

class Undistorter{

public:
    inline void apply(const cv::Mat &in,cv::Mat &out){
        assert(!mapx.empty());
        cv::remap(in,out,mapx,mapy,cv::INTER_LINEAR);

    }
inline void operator()(const cv::Mat &in,cv::Mat &out)    {
    assert(!mapx.empty());
    cv::remap(in,out,mapx,mapy,cv::INTER_LINEAR);

}
    void setParams(const reslam::ImageParams &ip,const cv::Mat &im){
        assert(ip.isValid());
        bool do_=false;

        if(!imgP.isValid()  || imgP!=ip  || imgP.CamSize!=im.size()){
            imgP=ip;
            imgP.resize(im.size());
            cv::initUndistortRectifyMap(imgP.CameraMatrix,imgP.Distorsion,cv::Mat(),cv::Mat(),imgP.CamSize,CV_32FC1,mapx,mapy);
        }
    }

    cv::Mat getImage(const reslam::Frame &kf,cv::VideoCapture &vcap ){
        vcap.set(CV_CAP_PROP_POS_FRAMES,kf.fseq_idx);//go to image
        vcap.grab();
        cv::Mat image;
        vcap.retrieve(image);
        //scale to
        image=resize(image,kf.imageParams.CamSize);
        setParams(kf.imageParams,image);
        cv::Mat res;
        apply(image,res);
        return res;
    }
private:
    cv::Mat mapx,mapy;
    reslam::ImageParams imgP;

    cv::Mat resize(cv::Mat im,cv::Size size){
        if( size==im.size() )return im;
        cv::Mat aux;
        cv::resize(im,aux, size);
        return aux;
    }
};

uint32_t getNextMostConnectedKeyFrame(reslam::Map &Map, const std::set<uint32_t> &excludedKfs){
    //first, take as first frame the one with more links to other keyframes
    std::pair<uint32_t,int> kf_neigh={std::numeric_limits<uint32_t>::max(),0};
    for(auto &kf:Map.keyframes){
        if(excludedKfs.count(kf.idx)!=0) continue;
        auto nn=Map.getNeighborKeyFrames(kf.idx,false);
        if( nn.size()>kf_neigh.second)kf_neigh={kf.idx,nn.size()};
    }
    return kf_neigh.first;
}
uint32_t getNextMostConnectedKeyFrameTo(uint32_t kfidx,reslam::Map &Map, const std::set<uint32_t> &excludedKfs){
    //first, take as first frame the one with more links to other keyframes
    std::pair<uint32_t,int> kf_neigh={std::numeric_limits<uint32_t>::max(),0};
    auto nn=Map.getNeighborKeyFrames(kfidx,false);
    for(auto n:nn){
        if(excludedKfs.count(n)!=0)continue;
        if( Map.TheKpGraph.getWeight(kfidx,n) > kf_neigh.second)
            kf_neigh={n,Map.TheKpGraph.getWeight(kfidx,n)};
    }
    return kf_neigh.first;
}

cv::Mat imscale(cv::Mat im,float scale){
        if( fabs(scale-1.0)<1e-3)return im;
            cv::Mat aux;
            cv::resize(im,aux, cv::Size ( float(im.cols)*scale,float(im.rows)*scale));
            return aux;
}

struct MapPointInfo{
    uint32_t id;
    cv::Point2f proj1 ;//projction in frame 1
    cv::Point2f proj2 ;

    MapPointInfo &operator*=(float sc){
        proj1*=sc;
        proj2*=sc;
        return *this;
    }
};

std::vector< MapPointInfo> getMapPointsInTwoFrames(reslam::Map &map, uint32_t f1,uint32_t f2){

    std::vector< MapPointInfo> res;
    auto kf1=map.keyframes[f1];
    auto kf2=map.keyframes[f2];

    for(size_t i=0;i< kf1.ids.size();i++){
        auto pi=kf1.ids[i];
        if(pi==std::numeric_limits<uint32_t>::max())continue;
        if( kf1.und_kpts[i].octave!=0)continue;

        if(  !map.map_points[pi].isObservingFrame(f2)) continue;
        auto obs=map.map_points[pi].getObservingFrames();
        //find the f2
        cv::KeyPoint kp2;kp2.octave=0;
        for(auto ff2:obs){
            if(ff2.first==f2) {
                kp2=kf2.und_kpts[ff2.second];
                break;
            }
        }
        if(kp2.octave!=0)continue;
        MapPointInfo mpi;
        mpi.id=pi;
        mpi.proj1=kf1.und_kpts[i].pt;
        mpi.proj2=kp2.pt;
        res.push_back(mpi);

    }
    return res;
}

void imshow(string name,cv::Mat image,cv::Size size){
    cv::Mat im;
    resize(image,im,size);
    cv::imshow(name,im);
}

void blend(cv::Mat &inOut,cv::Mat &in2){
    for(int y=0;y<inOut.rows;y++){
        cv::Vec3b *inout_ptr=inOut.ptr<cv::Vec3b>(y);
        cv::Vec3b *in2_ptr=in2.ptr<cv::Vec3b>(y);
        for(int x=0;x<inOut.cols;x++){
            if( inout_ptr[x][0]==0 && inout_ptr[x][1]==0 && inout_ptr[x][2]==0)
                inout_ptr[x] =in2_ptr[x];
        }
    }
}

int main(int argc,char **argv){
    try {
        if(argc<4)throw std::runtime_error("Usage: map video outDir [-scale <value>]");

        CmdLineParser cml(argc,argv);
        float scale=stof(cml("-scale","1"));

        cout<<"Opening Map"<<endl;
        reslam::Map Map;
        Map.readFromFile(argv[1]);

        cout<<"Opening Video"<<endl;
        cv::VideoCapture vcap;
        vcap.open(argv[2]);
        if(!vcap.isOpened()) throw std::runtime_error("Could not open:"+string(argv[2]));


        std::map<uint32_t,cv::Mat> images;
        cv::Mat image,undImage;
        int imgIdx=0;

        Undistorter undist;
        std::set<uint32_t> usedKeyFrames;


        uint32_t  cur_kf_id=getNextMostConnectedKeyFrame(Map,usedKeyFrames);
        usedKeyFrames.insert(cur_kf_id);


        //first, take as first frame the one with more links to other keyframes
        cv::Mat Mosaic= undist.getImage(Map.keyframes[cur_kf_id],vcap);
        cv::Mat prevH=(cv::Mat_<float>(2,3)<<1,0,0,0,1,0);
        do {
            auto kf_id=getNextMostConnectedKeyFrameTo(cur_kf_id,Map,usedKeyFrames);
            if(kf_id==std::numeric_limits<uint32_t>::max()) break;
            //join both
            auto points=getMapPointsInTwoFrames(Map,cur_kf_id,kf_id);
            //draw matches
            cv::Mat imckf2_= undist.getImage(Map.keyframes[kf_id],vcap);

            vector<cv::Point2f> p1,p2;
            for(auto p:points){
                p1.push_back(p.proj1);
                p2.push_back(p.proj2);
            }
            //move the points p1 according to current transform
            vector<cv::Point2f> p1_inmosaic;
            cv::transform(p1,p1_inmosaic,prevH);
            cv::Mat curH=cv::estimateAffinePartial2D(p2,p1_inmosaic);
            cout<<"curH"<<endl<<curH<<endl;
            //find the location of the image corners
            vector<cv::Point2f> corners={ cv::Point2f(0,0), cv::Point2f (0,imckf2_.cols),
                                          cv::Point2f(imckf2_.rows,imckf2_.cols),cv::Point2f(imckf2_.rows,0)};

            vector<cv::Point2f> corners2;
            cv::transform(corners,corners2,curH);
            //get min and max corners if any
            cv::Point2f minP(std::numeric_limits<int>::max(),std::numeric_limits<int>::max());
            cv::Point2f maxP(std::numeric_limits<int>::lowest(),std::numeric_limits<int>::lowest());
            for(auto c:corners2){
                minP.x=std::min(minP.x, c.x);
                minP.y=std::min(minP.y, c.y);
                maxP.x=std::max(maxP.x, c.x);
                maxP.y=std::max(maxP.y, c.y);
            }
            cout<<minP<<" "<<maxP<<endl;cout<<"||||||||||||||||||||"<<endl;
            if(maxP.x>= Mosaic.cols || maxP.y>=Mosaic.rows){
                cv::Mat aux(std::max( int(maxP.y+0.5)+1,Mosaic.rows), std::max(int(maxP.x+0.5)+1,Mosaic.cols) , Mosaic.type());
                aux=0;
                cv::Mat region=aux.rowRange(0,Mosaic.rows).colRange(0,Mosaic.cols);
                Mosaic.copyTo(region);
                Mosaic=aux;
            }

            cv::Mat warped;
            cv::warpAffine(imckf2_,warped,curH,Mosaic.size());

            cout<<corners2<<endl;
            cout<<points[2].proj1<<" "<<points[2].proj2<<endl;
             imshow("im1",Mosaic,cv::Size(800,600));
            imshow("im2",imckf2_,cv::Size(800,600));
            imshow("im3",warped,cv::Size(800,600));
            blend(Mosaic,warped);
            imshow("res",Mosaic,cv::Size(800,600));
            while( cv::waitKey(0)!='n') ;


            usedKeyFrames.insert(cur_kf_id);
            cur_kf_id=kf_id;
            prevH=curH.clone();
        }while(usedKeyFrames.size()<Map.keyframes.size() );
        exit(0);

        //find the one with most
//        std::pair<uint64_t,float> kf_neigh={std::numeric_limits<uint64_t>::max(),0};

//        for(auto n:nn)
//            if( Map.TheKpGraph.getEdge(kf.idx,n) >  kf_neigh.second) kf_neigh={kf.idx,Map.TheKpGraph.getEdge(kf.idx,n)};

        //now, look for the neighbor with most

        for(auto &kf:Map.keyframes){
            cout<<"processing "<<imgIdx<<"/"<<Map.keyframes.size()<<" ";
            cout.flush();
            usedKeyFrames.insert(kf.idx);
            vcap.set(CV_CAP_PROP_POS_FRAMES,kf.fseq_idx);//go to image
            vcap.grab();
            vcap.retrieve(image);
            undist.setParams(kf.imageParams,image);
            undist(image,undImage);

             imgIdx++;/*
             if(imgIdx>=maxImages) break;*/
        }


    } catch (std::exception &ex) {
        cout<<ex.what()<<endl;
    }
}

//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/stitching.hpp"
//#include <iostream>
//using namespace std;
//using namespace cv;
//bool divide_images = false;
//Stitcher::Mode mode = Stitcher::PANORAMA;
//vector<Mat> imgs;
//string result_name = "result.jpg";
//void printUsage(char** argv);
//int parseCmdArgs(int argc, char** argv);
//int main(int argc, char* argv[])
//{
//    int retval = parseCmdArgs(argc, argv);
//    if (retval) return EXIT_FAILURE;
//    Mat pano;
//    Ptr<Stitcher> stitcher = Stitcher::create(mode);
//    Stitcher::Status status = stitcher->stitch(imgs, pano);
//    if (status != Stitcher::OK)
//    {
//        cout << "Can't stitch images, error code = " << int(status) << endl;
//        return EXIT_FAILURE;
//    }
//    imwrite(result_name, pano);
//    cout << "stitching completed successfully\n" << result_name << " saved!";
//    return EXIT_SUCCESS;
//}
//void printUsage(char** argv)
//{
//    cout <<
//         "Images stitcher.\n\n" << "Usage :\n" << argv[0] <<" [Flags] img1 img2 [...imgN]\n\n"
//         "Flags:\n"
//         "  --d3\n"
//         "      internally creates three chunks of each image to increase stitching success\n"
//         "  --mode (panorama|scans)\n"
//         "      Determines configuration of stitcher. The default is 'panorama',\n"
//         "      mode suitable for creating photo panoramas. Option 'scans' is suitable\n"
//         "      for stitching materials under affine transformation, such as scans.\n"
//         "  --output <result_img>\n"
//         "      The default is 'result.jpg'.\n\n"
//         "Example usage :\n" << argv[0] << " --d3 --try_use_gpu yes --mode scans img1.jpg img2.jpg\n";
//}
//int parseCmdArgs(int argc, char** argv)
//{
//    if (argc == 1)
//    {
//        printUsage(argv);
//        return EXIT_FAILURE;
//    }
//    for (int i = 1; i < argc; ++i)
//    {
//        if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
//        {
//            printUsage(argv);
//            return EXIT_FAILURE;
//        }
//        else if (string(argv[i]) == "--d3")
//        {
//            divide_images = true;
//        }
//        else if (string(argv[i]) == "--output")
//        {
//            result_name = argv[i + 1];
//            i++;
//        }
//        else if (string(argv[i]) == "--mode")
//        {
//            if (string(argv[i + 1]) == "panorama")
//                mode = Stitcher::PANORAMA;
//            else if (string(argv[i + 1]) == "scans")
//                mode = Stitcher::SCANS;
//            else
//            {
//                cout << "Bad --mode flag value\n";
//                return EXIT_FAILURE;
//            }
//            i++;
//        }
//        else
//        {
//            Mat img = imread(samples::findFile(argv[i]));
//            if (img.empty())
//            {
//                cout << "Can't read image '" << argv[i] << "'\n";
//                return EXIT_FAILURE;
//            }
//            if (divide_images)
//            {
//                Rect rect(0, 0, img.cols / 2, img.rows);
//                imgs.push_back(img(rect).clone());
//                rect.x = img.cols / 3;
//                imgs.push_back(img(rect).clone());
//                rect.x = img.cols / 2;
//                imgs.push_back(img(rect).clone());
//            }
//            else
//                imgs.push_back(img);
//        }
//    }
//    return EXIT_SUCCESS;
//}
