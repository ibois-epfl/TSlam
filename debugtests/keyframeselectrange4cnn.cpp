#include <iostream>
#include "utils/framematcher.h"
#include "utils/frameextractor.h"
#include "basictypes/timers.h"
#include "basictypes/misc.h"
#include "map_types/covisgraph.h"
#include "basictypes/se3transform.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
using namespace std;

cv::Mat compose(const vector<reslam::Frame>& frames,const vector<cv::DMatch> &matches,int idx=-1){

    auto randColor=[](){  return int(255.*float(rand())/float(RAND_MAX));  };

    cv::Mat composed,colorComposed;
    composed.create(frames[0].image.rows,frames.size()*frames[0].image.cols,frames[0].image.type());
    for(size_t i=0;i<frames.size();i++){
        auto subIm=composed.rowRange(0,frames[0].image.rows).colRange(i*frames[0].image.cols,(i+1)*frames[0].image.cols);
        frames[i].image.copyTo(subIm);
    }
    cv::cvtColor(composed,colorComposed,CV_GRAY2BGR);
    //now, draw
    if(idx==-1) idx=1;
    idx=idx%matches.size();
    auto kp1=frames[0].kpts[ matches[idx].trainIdx ];
    auto kp2=frames[1].kpts[ matches[idx].queryIdx ]+cv::Point2f(frames[0].image.cols,0);
    cv::line( colorComposed,kp1,kp2,cv::Scalar(randColor(),randColor(),randColor()));

    return  colorComposed;
}

void readFrame( cv::VideoCapture &vcap, uint32_t fidx, const reslam::ImageParams &ip,reslam::FrameExtractor &Fext ,reslam::Frame &Frame,const map<int,cv::Mat>& poses){

    cv::Mat im;
    vcap.set(CV_CAP_PROP_POS_FRAMES,fidx);
    vcap.grab();
    vcap.set(CV_CAP_PROP_POS_FRAMES,fidx);
    vcap.grab();
    vcap.retrieve(im);
    Fext.process(im,ip,Frame,fidx);
    Frame.pose_f2g=poses.at(fidx);
}




map<int,cv::Mat> readGtFile(const string &path,bool invert){
    auto getMatrix=[](double qx,double qy, double qz,double qw,double tx,double ty ,double tz){
        double qx2 = qx*qx;
        double qy2 = qy*qy;
        double qz2 = qz*qz;
        cv::Mat m=cv::Mat::eye(4,4,CV_64F);
        m.at<double>(0,0)=1 - 2*qy2 - 2*qz2;
        m.at<double>(0,1)=2*qx*qy - 2*qz*qw;
        m.at<double>(0,2)=2*qx*qz + 2*qy*qw;
        m.at<double>(0,3)=tx;
        m.at<double>(1,0)=2*qx*qy + 2*qz*qw;
        m.at<double>(1,1)=1 - 2*qx2 - 2*qz2;
        m.at<double>(1,2)=2*qy*qz - 2*qx*qw;
        m.at<double>(1,3)=ty;
        m.at<double>(2,0)=2*qx*qz - 2*qy*qw	;
        m.at<double>(2,1)=2*qy*qz + 2*qx*qw	;
        m.at<double>(2,2)=1 - 2*qx2 - 2*qy2;
        m.at<double>(2,3)=tz;
        return m;
    };

    ifstream file(path);
    if(!file)throw std::runtime_error("Could not open file:"+path);
    map<int,cv::Mat> fmap;
    float tx,ty,tz,qx,qy,qz,qw;
    string stamp;
    cv::Mat firstFrameT;
    while(!file.eof()){
        string line;
        std::getline(file,line);
        stringstream sline;sline<<line;
        if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
            auto m=getMatrix(qx,qy,qz,qw,tx,ty,tz);
            if (invert)m=m.inv();
            m.convertTo(m,CV_32F);
            fmap.insert( {stoi(stamp),  m});
        }
    }
    return fmap;
}


vector<cv::DMatch> findMatches(const reslam::Frame &f0,const reslam::Frame &f1,float &acos,float scaleFactor=1.2){
    //compute
    cv::Mat Q2T= f0.pose_f2g.inv()  * f1.pose_f2g;
    reslam::Se3Transform TN_G2F=f0.pose_f2g.inv();//matrix moving points from the new frame to the global ref system

    cv::Point3f zero=reslam::mult<float>(Q2T,cv::Point3f(0,0,0));
    cv::Point3f one=reslam::mult<float>(Q2T,cv::Point3f(0,0,1));
    cv::Point3f norm=one-zero;

      acos= norm.dot(cv::Point3f(0,0,1));

    std::vector<cv::DMatch> matches;
    if(acos>0.5){
        //find matches
        reslam::FrameMatcher FMatch;
        FMatch.setParams(f0,reslam::FrameMatcher::MODE_ALL,80);
        matches= FMatch.matchEpipolar(f1,reslam::FrameMatcher::MODE_ALL,Q2T);

        vector<cv::Point3f> p3d= reslam::Triangulate(f0,f1,Q2T.inv(),matches);
        ///save the good matches and move the point the frame ref system
        float ratioFactor=1.5f*scaleFactor;
        int nnan=0;
        for(size_t i=0;i<matches.size();i++){
            if (!isnan( p3d[i].x)){
                //check ratioDist
                cv::Point3f p3global=TN_G2F*p3d[i];
                //distance to frames
                float distNF=cv::norm(p3global-f0.getCameraCenter());
                float distF2=cv::norm(p3global-f1.getCameraCenter());
                if(distNF==0 || distF2==0){
                    matches[i].trainIdx=-1;
                    continue;
                }
                const float ratioDist = distNF/distF2;
                int oct_NewFrame=   f0.und_kpts[ matches[i].trainIdx].octave;
                int oct_frame2=   f1.und_kpts[ matches[i].queryIdx].octave;
                const float ratioOctave =f0.scaleFactors[oct_NewFrame] /f1.scaleFactors[oct_frame2];
                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor){
                    matches[i].trainIdx=-1;
                    continue;
                }
            }
            else{
                nnan++;
                matches[i].trainIdx=-1;
            }
        }

        reslam::remove_unused_matches(matches);
     }
    return matches ;
}
struct triple{
    int a,b,c;
};

void printTransform(reslam::Se3Transform &Se3){
    cv::Mat rvec=Se3.getRvec(),tvec=Se3.getTvec();
    for(int p=0;p<3;p++) cout<<rvec.ptr<float>(0)[p]<<" ";
    for(int p=0;p<3;p++) cout<<tvec.ptr<float>(0)[p]<<" ";
 }

int main(int argc,char **argv){

    try {

        if( argc!=7)throw std::runtime_error(" video.mp4 camparams.yml gt kfStart kfEND MinMatches");

        cv::VideoCapture vcap;
        vcap.open(argv[1]);
        if(!vcap.isOpened())throw std::runtime_error("Could not open video");

        reslam::ImageParams imParams;
        imParams.readFromXMLFile(argv[2]);

        auto featuresDetector=reslam::Feature2DSerializable::create(reslam::DescriptorTypes::DESC_ORB);
        reslam::FrameExtractor Fextra;
        reslam::Params params;
        params.maxFeatures=5000;
        params.detectMarkers=false;
        Fextra.setParams(featuresDetector,params);
        Fextra.saveImage()=true;
        vector<int> indices;
        int start=stoi(argv[4]);
        int end=stoi(argv[5]);
        int minMatches=stoi(argv[6]);
        int numFrames=vcap.get(CV_CAP_PROP_FRAME_COUNT);
        auto GtPoses=readGtFile(argv[3],false);
        for(int i=start;i<end;i++){

            if (i>=numFrames)throw std::runtime_error("Index of frames out of bounds");

            if( GtPoses.count(i)!=0) indices.push_back(i);

        }

        if( indices.size()<2) return -1;

        vector<reslam::Frame> Frames(indices.size());
        for(size_t i=0;i<indices.size();i++)
            readFrame( vcap,indices[i],imParams,Fextra,Frames[i],GtPoses);

        std::map<uint64_t,reslam::Se3Transform> frames_transform;
        std::set<int> frames;
        std::vector<pair<uint32_t,uint32_t> >NoMinMatchesButValid;


        float cosangle;
//        vector<tuple<int,int,> > pairData;//frame1 frame2   transfor
        for(size_t i=0;i<indices.size();i++)
            for(size_t j=i+1;j<indices.size();j++){
                auto matches=findMatches(Frames[i],Frames[j],cosangle,params.scaleFactor);
                if(cosangle<0.5) continue;
                if(matches.size()<minMatches  ) continue;


                cv::Mat i2j= Frames[i].pose_f2g.inv()  * Frames[j].pose_f2g;
                reslam::Se3Transform Se3i2j;Se3i2j=i2j;
                frames_transform.insert( {reslam::CovisGraph::join(i,j),Se3i2j});
                frames.insert( i);
                frames.insert(j);
            }

       std::vector<int> vframes;
       for(auto i:frames) vframes.push_back(i);


       for(size_t i=0;i<vframes.size();i++)
           for(size_t j=0;j<vframes.size();j++)
               for(size_t k=0;k<vframes.size();k++){
                   if( i==j || i==k || j==k) continue;
                   if( frames_transform.count(reslam::CovisGraph::join(i,j)) &&
                           frames_transform.count(reslam::CovisGraph::join(i,k)) &&
                           frames_transform.count(reslam::CovisGraph::join(j,k))
                           ){
                       cout<<indices[i]<<" "<<indices[j]<<" "<<indices[k]<<" ";

                       auto Se3=frames_transform.at(reslam::CovisGraph::join(i,j));
                       if( i>j) Se3=Se3.inv();
                       printTransform(Se3);

                       Se3=frames_transform.at(reslam::CovisGraph::join(i,k));
                       if( i>k) Se3=Se3.inv();
                        printTransform(Se3);

                       cout<<endl;

                   }
           }

    } catch (std::exception &ex) {
        cout<<ex.what()<<endl;
    }

}
