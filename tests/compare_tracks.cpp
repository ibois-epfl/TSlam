#include <fstream>
#include <iostream>

#include "logtools.h"

#include <math.h>
#include <basictypes/misc.h>
#include <iomanip>

using namespace std;
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

cv::Point2d get2dT(cv::Mat &m){
    if(m.type()==CV_64F)
        return cv::Point2d(m.at<double>(0,3),m.at<double>(1,3));
    else
        return cv::Point2d(m.at<float>(0,3),m.at<float>(1,3));
}

cv::Point3d getT(cv::Mat &m){
    if(m.type()==CV_64F)
        return cv::Point3d(m.at<double>(0,3),m.at<double>(1,3),m.at<double>(2,3));
    else
        return cv::Point3d(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3));
}

std::map<string,cv::Point3d> loadControlPoints(string path)
{
    std::map<string,cv::Point3d> res;
    std::ifstream file(path,ios::binary);
    if(!file) throw std::runtime_error("Could not open file:"+path);
    while(!file.eof()){
        string line;
        std::getline(file,line);
        stringstream sline;sline<<line;

        string frame;
        cv::Point3f pos;

        if (sline>>frame>>pos.x>>pos.y>>pos.z){
           res.insert( {frame, pos});//refers everything to the first frame
       }
    }

    return res;
}

std::map<string,cv::Point2d> loadGTFile(string path)
{
    std::map<string,cv::Point2d> res;
    std::ifstream file(path,ios::binary);
    if(!file) throw std::runtime_error("Could not open file:"+path);

    while(!file.eof()){
        string line;
        std::getline(file,line);
        stringstream sline;sline<<line;



        string id;
        float distance, angle;
        cv::Point2d pos;

        if (sline>>id>>distance>>angle>>pos.x>>pos.y){
           res.insert( {id, pos});//refers everything to the first frame
       }
    }

    return res;
}

void centerMap(std::map<string, cv::Point3d>& controlPnts){
    cv::Point3d avrg(0,0,0);
    for(auto cm:controlPnts) avrg+=cm.second;
    avrg*=1./double(controlPnts.size());
    for(auto &cm:controlPnts) {cm.second-=avrg;
    }
}
void centerMap(std::map<string, cv::Point2d>& controlPnts){
    cv::Point2d avrg(0,0);
    for(auto cm:controlPnts) avrg+=cm.second;
    avrg*=1./double(controlPnts.size());
    for(auto &cm:controlPnts) {cm.second-=avrg;
    }
}

vector<cv::Vec4f>  toPcd(  map<string,cv::Mat> &data,cv::Scalar color){
   auto toVec=[](   map<string,cv::Mat> &data){
        map<int,cv::Point3d> datai;
        for(auto d:data) datai.insert( { std::stoi(d.first),getT(d.second)});
        vector<pair<int,cv::Point3d> > res;
        for(auto d:datai) res.push_back({d.first,d.second});
        return res;
    };
   vector<cv::Vec4f> points;
   auto gtPos=toVec(data);
   int start=1,end=gtPos.size();
   for(size_t i=start;i<end;i++){
        auto pp=reslam::logtools::getLine(gtPos[i-1].second,gtPos[i].second,color,1);
       points.insert(points.end(),pp.begin(),pp.end());
   }
   return points;
}

float alignAndScaleToGroundTruth(std::map<string, cv::Mat>& log, std::map<string, cv::Point3d> controlPnts)
{
    vector<cv::Point3f> src,dst;

    for(auto control:controlPnts){
        auto pos=log.find(control.first);
        if(pos!=log.end())
        {
            src.push_back( cv::Point3d(pos->second.at<double>(0,3),pos->second.at<double>(1,3),pos->second.at<double>(2,3)) );
            dst.push_back( control.second);
        }
    }
    if(src.size()<2)
        return -1;



    double err=0;
    auto T=reslam::rigidBodyTransformation_Horn1987(src,dst,false,&err);
    cv::Mat T64;
    if( T.type()!=CV_64F) T.convertTo(T64,CV_64F);
    else T64=T;

    for(auto &gto:log)
         gto.second = T64 * gto.second;

    return err;
}
cv::Mat  getMatrix(double qx,double qy, double qz,double qw,double tx,double ty ,double tz){


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
}
map<string,cv::Mat>   loadFile(std::string fp){
  map<string,cv::Mat>   fmap;
   ifstream file(fp);
   if(!file)throw std::runtime_error("Could not open file:"+fp);
   string stamp;
   float tx,ty,tz,qx,qy,qz,qw;
   cv::Mat firstFrameT;
   while(!file.eof()){
       string line;
       std::getline(file,line);
       stringstream sline;sline<<line;
        if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
            auto m=getMatrix(qx,qy,qz,qw,tx,ty,tz);
           fmap.insert( {stamp, m});//refers everything to the first frame
       }
    }

   //now, find the transform from every frame to the first
   return fmap;
}

int main(int argc,char **argv){

    CmdLineParser cml(argc,argv);
    if(argc<4){
        cerr<<"Usage: MapSeqLog MapSeqControlPnts TrckSeqLog TrckSeqControlPnts [-gt trajectory] [-pcd]"<<endl;return -1;
    }

    auto mapControlPnts=loadControlPoints(argv[2]);

    //The last control point is not used.
    int nframesMap=-1;
    for(auto control:mapControlPnts)
        if(stoi(control.first)>nframesMap)nframesMap=stoi(control.first);
    mapControlPnts.erase(to_string(nframesMap));
    nframesMap++; //Because start by 0

    map<string,cv::Mat> mapPoses;
    double errHornMapPnt=-1;
    ifstream maplogfile(argv[1]);
    if(maplogfile)
    {
        mapPoses=loadFile(argv[1]);
        errHornMapPnt = alignAndScaleToGroundTruth(mapPoses, mapControlPnts);
    }


    auto slamControlPnts=loadControlPoints(argv[4]);
    int nframesTrck=-1;
    for(auto control:slamControlPnts)
        if(stoi(control.first)>nframesTrck)nframesTrck=stoi(control.first);
    slamControlPnts.erase(to_string(nframesTrck));
    nframesTrck++; //Because start by 0

    ifstream slamlogfile(argv[3]);
    map<string,cv::Mat> slamPoses;
    double errHornTrckPnt=-1;
    if(slamlogfile)
    {
        slamPoses=loadFile(argv[3]);
        errHornTrckPnt = alignAndScaleToGroundTruth(slamPoses, slamControlPnts);
    }



    //Load ground truth file
    auto gt=loadGTFile(cml("-gt"));




    vector<cv::Vec4f> first = toPcd(mapPoses,cv::Scalar(0,0,255));
    vector<cv::Vec4f> second = toPcd(slamPoses,cv::Scalar(255,0,0));
    first.insert( first.end(), second.begin(), second.end() );
    if(cml["-pcd"])reslam::logtools::savePcd("both.pcd",first);


    //Error trajectory between map and tracking
    std::pair<int, double> err1(0,0);
    for(auto f2:slamPoses)
    {
        double dst = std::numeric_limits<double>::max();
        for(auto f1:mapPoses)
        {
            double diff = cv::norm(get2dT(f2.second)-get2dT(f1.second));
            if(diff < dst)
                dst=diff;
        }
        err1.first++;
        err1.second+=dst;
    }

    //Error trajectory between map and gt
    std::pair<int, double> err2(0,0);
    for(auto f2:slamPoses)
    {
        double dst = std::numeric_limits<double>::max();
        for(auto f1:gt)
        {
            double diff = cv::norm(get2dT(f2.second)-f1.second);
            if(diff < dst)
                dst=diff;
        }
        err2.first++;
        err2.second+=dst;
    }


    //Error trajectory between map and gt
    std::pair<int, double> err3(0,0);
    for(auto pose:mapPoses)
    {
        double dst = std::numeric_limits<double>::max();
        for(auto f1:gt)
        {
            double diff = cv::norm(get2dT(pose.second)-f1.second);
            if(diff < dst)
                dst=diff;
        }
        err3.first++;
        err3.second+=dst;
    }
    std::cout << "map_horn_err,trck_horn_err,trck_map_err,trck_map_n,trck_gt_err,trck_gt_n,map_gt_err,map_gt_n"<<std::endl;
    std::cout   << errHornMapPnt<< "," <<errHornTrckPnt<<","
                << err1.second/err2.first << "," <<err1.first*100.f/float(nframesTrck)<<","
                << err2.second/err2.first << "," <<err2.first*100.f/float(nframesTrck)<<","
                << err3.second/err3.first << "," <<err3.first*100.f/float(nframesMap)<<std::endl;
}

