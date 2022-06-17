#include <map>
#include <set>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iomanip>
#include "dirreader.h"
#include "logtools.h"

#include <basictypes/misc.h>
#include <filesystem>

using namespace  std;
using namespace  reslam;


struct SeqTestInfo{
    std::string seq_name,cam,method,cam_id;
    std::string map_seq_name, map_cam, map_cam_id;

    std::string fullPath;
    std::string gt_file;
    std::string controlPnts;
    std::string time_file;

    std::string getKey()const{
        return seq_name+cam+method;
    }
    std::string getDesc()const{
        return cam+"|"+seq_name+"|"+cam_id+"_"+map_cam+"|"+map_seq_name+"|"+map_cam_id;
    }
};

std::vector<SeqTestInfo> _getDataSetTestInfo(string resultsPath_dataset, string TheDataSetPath_dataset ){

    auto  parseFn=[](filesystem::path fileName,SeqTestInfo &ti){
                ti.fullPath=fileName;
                string baseName = fileName.parent_path().filename();

                auto pos=baseName.find_last_of('_');
                string slamBaseName=std::string(baseName.begin(),baseName.begin()+pos);
                string mapBaseName=std::string(baseName.begin()+pos+1,baseName.end());

//                std::cout << slamBaseName <<", "<<mapBaseName<<std::endl;

                std::replace( slamBaseName.begin(), slamBaseName.end(), '-', ' '); // replace all 'x' to 'y'
                std::replace( mapBaseName.begin(), mapBaseName.end(), '-', ' '); // replace all 'x' to 'y'


                stringstream sstr, sstr2;sstr<<slamBaseName; sstr2<<mapBaseName;
                if ( (sstr>>ti.cam>>ti.seq_name>> ti.cam_id) and (sstr2>>ti.map_cam>>ti.map_seq_name>> ti.map_cam_id))
                    return true;
                else
                    return false;
//                auto pos=fileName.find_last_of('/')+1;
//                string baseName=std::string(fileName.begin()+pos,fileName.end());

//                std::replace( baseName.begin(), baseName.end(), '@', ' '); // replace all 'x' to 'y'
//                std::replace( baseName.begin(), baseName.end(), '.', ' '); // replace all 'x' to 'y'

//                stringstream sstr;sstr<<baseName;


//                if ( sstr>>ti.seq_name>>ti.cam>> ti.iteration)
//                    return true;
//                else return false;
                //mh_01@cam0@2.log

            };

    std::vector<filesystem::path>files;
    try {
        for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(resultsPath_dataset))
             if(dirEntry.path().filename()=="poses.txt")
             {
                 files.push_back(dirEntry.path());
             }

    }  catch (std::exception e) {}

    std::vector<SeqTestInfo> tests;
    for(auto file:files){
//        if(file.find("@")==std::string::npos)continue;
        SeqTestInfo ti;
        if(parseFn(file,ti)){
            //set the gt file path
//            ti.gt_file=TheDataSetPath_dataset+"/"+ti.seq_name+"/"+ti.cam+"trajectory.txt";
            ti.controlPnts=TheDataSetPath_dataset+"/"+ti.cam+"/"+ti.seq_name+"/control.txt";
            ti.gt_file=TheDataSetPath_dataset+"/gt/trajectory.txt";
//            std::cout << ti.controlPnts<<std::endl;
//            std::cout << ti.gt_file<<std::endl;
//            if( std::find( filesTime.begin(),filesTime.end(),file+"time")!=filesTime.end()){
//                ti.time_file=file+"time";
//            }
            tests.push_back(ti);
        }
    }
    return tests;
}

struct FrameMatchLocation{
     cv::Point2d first;
     cv::Point2d second;
     std::string frame;
     double error;

};

struct TestResult{
    SeqTestInfo ti;
    double ATE=-1;
     double perctFramesTracked=0;

     std::vector< FrameMatchLocation > matchedFrames;//errors in each of the frames
     bool isFrame(std::string frame){
         for(auto &f:matchedFrames)
             if (f.frame==frame)return true;
         return false;
     }
     std::map<std::string,cv::Point2d> gt;
     std::map<std::string,cv::Mat> poses;
     int64_t timeMapping=-1,timeTracking=-1;
     float getMappingFPS(){if (timeMapping<=1 )return -1; return (gt.size()*perctFramesTracked/100.)/double(timeMapping);}
     float getTrackingFPS(){if (timeTracking<=1)return -1; return (gt.size()*perctFramesTracked/100.)/double(timeTracking);}

};

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

float alignAndScaleToGroundTruth(std::map<string, cv::Mat>& log, std::map<string, cv::Point3d> controlPnts)
{
    vector<cv::Point3f> src,dst;

    for(auto control:controlPnts){
        auto pos=log.find(control.first);
        if(pos!=log.end())
        {
            src.push_back( cv::Point3d(pos->second.at<double>(0,3),pos->second.at<double>(1,3),pos->second.at<double>(2,3)) );
            dst.push_back( control.second);
            //std::cout <<"["<< control.first<<"] "<<src.back() <<", dst: "<<dst.back()<< std::endl;
        }
    }
//    std::cout <<"Size:"<<src.size() << std::endl;
    if(src.size()<3)
        return -1;


    //SCALE THE MODEL
//    double scale=0;
//    {
//    int ntimes=0;
//    for(size_t i=0;i<src.size();i++){
//        for(size_t j=i+1;j<src.size();j++){
//            double distOrg=cv::norm(src[i]-src[j]);
//            double distDst=cv::norm(dst[i]-dst[j]);
//            scale+=  distOrg/distDst;
//            ntimes++;
//        }
//     }
//    scale/=double(ntimes);
//    }
//    cout<<"scale="<<scale<<endl;
//    for(auto &o:src) o*=scale;

    double err=0;
    auto T=reslam::rigidBodyTransformation_Horn1987(src,dst,false,&err);
    //auto T= ucoslam::logtools::rigidBodyTransformation_Horn1987(src,dst,false);
    cv::Mat T64;
    if( T.type()!=CV_64F) T.convertTo(T64,CV_64F);
    else T64=T;

    for(auto &gto:log)
         gto.second = T64 * gto.second;


    return err;
}

cv::Point2d get2dT(cv::Mat &m){
    if(m.type()==CV_64F)
        return cv::Point2d(m.at<double>(0,3),m.at<double>(1,3));
    else
        return cv::Point2d(m.at<float>(0,3),m.at<float>(1,3));
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
TestResult _analyzeTest(SeqTestInfo &ti){

    TestResult tr;
    tr.ti=ti;
    tr.gt=loadGTFile(ti.gt_file);

    tr.poses=loadFile(ti.fullPath);
    auto slamControlPnts=loadControlPoints(ti.controlPnts);

    //Cal nframesTrack
    int nframesTrck=-1;
    for(auto control:slamControlPnts)
        if(stoi(control.first)>nframesTrck)nframesTrck=stoi(control.first);
    slamControlPnts.erase(to_string(nframesTrck));
    nframesTrck++; //Because start by 0

    float errHorn = alignAndScaleToGroundTruth(tr.poses, slamControlPnts);
//    std::cout << errHorn<<std::endl;

    for(auto pose:tr.poses)
    {
        FrameMatchLocation match;
        match.second = get2dT(pose.second);
        match.error=std::numeric_limits<double>::max();

        for(auto gt:tr.gt)
        {
            double diff = cv::norm(get2dT(pose.second)-gt.second);
            if(diff < match.error)
            {
                match.frame=pose.first;
                match.first=gt.second;
                match.error=diff;
            }
        }

        if(match.error<std::numeric_limits<double>::max())
            tr.matchedFrames.push_back(match);
    }

//    tr.ATE=err3.second/err3.first;
//    tr.perctFramesTracked=100* double(err3.first)/double( nframesTrck);

//    tr.matchedFrames =ucoslam::logtools::getMatchedLocations(tr.gt,tr.poses);
//    if (tr.matchedFrames .size()==0) return tr;
//    ucoslam::logtools::alignAndScaleToGroundTruth(tr.matchedFrames );
//    //get the ATE error
//    double e=0;
//    for(auto &p:tr.matchedFrames )
//        e+=p.error=cv::norm(getT(p.first)-getT(p.second));

//    tr.ATE=e/double(tr.matchedFrames .size());
//    tr.perctFramesTracked=100* double(tr.matchedFrames .size())/double( tr.gt.size());
   // ucoslam::logtools::getMatchedLocations_io(tr.poses,tr.poses);


    if (tr.matchedFrames .size()==0) return tr;
    //get the ATE error
    double e=0;
    for(auto &p:tr.matchedFrames )
        e+=p.error=cv::norm(p.first-p.second);

    tr.ATE=e/double(tr.matchedFrames .size());
    tr.perctFramesTracked=100* double(tr.matchedFrames .size())/double( nframesTrck);

    return tr;
}

//returns the following values
//0: ATE of first method
//1: ATE of second method
//2: Perct of Frames tracket of first method
//3: Perct of Frames tracket of second method
struct comparison{
    std::string seq_name,cam,method1,method2;
    float ate1=std::numeric_limits<float>::max(),ate2=std::numeric_limits<float>::max(),track1=0,track2=0;
};

comparison  _compareMethods2(SeqTestInfo &t1,SeqTestInfo &t2)
{
    comparison comp;
    comp.seq_name=t1.seq_name;
    comp.cam=t1.cam;
    comp.method1=t1.method;
    comp.method2=t2.method;
    //analyze the whole sequence individually, and then, take the errors only in the common frames
    auto t1res=_analyzeTest(t1);
    auto t2res=_analyzeTest(t2);


    cout<<t1res.ATE<<" "<<t1res.perctFramesTracked<<" | ";
    cout<<t2res.ATE<<" "<<t2res.perctFramesTracked<<" | "<<std::endl;

    //intersect the frames tracked to recompute the ate with them
    for(auto it=t1res.matchedFrames.begin();it!=t1res.matchedFrames.end();){
        if (t2res.isFrame(it->frame)) it++;
        else it=t1res.matchedFrames.erase(it);
    }

    for(auto it=t2res.matchedFrames.begin();it!=t2res.matchedFrames.end();){
        if (t1res.isFrame(it->frame)) it++;
        else it=t2res.matchedFrames.erase(it);
    }

    if( t1res.matchedFrames.size()>0){

    //ucoslam::logtools::alignAndScaleToGroundTruth(t1res.matchedFrames );
    for(auto &p:t1res.matchedFrames )
        p.error=cv::norm(p.first-p.second);
    //ucoslam::logtools::alignAndScaleToGroundTruth(t2res.matchedFrames );
    for(auto &p:t2res.matchedFrames )
        p.error=cv::norm(p.first-p.second);



    //recompute ates
    double err=0;

    for(const auto &m:t1res.matchedFrames)
        err+=m.error;
    t1res.ATE=err/double(t1res.matchedFrames.size());
    err=0;
    for(const auto &m:t2res.matchedFrames)
        err+=m.error;
    t2res.ATE=err/double(t2res.matchedFrames.size());

    comp.ate1=t1res.ATE;
    comp.ate2=t2res.ATE;
    comp.track1=t1res.perctFramesTracked;
    comp.track2=t2res.perctFramesTracked;
     }
    else{
        t1res.ATE=0;
        t2res.ATE=0;
    }

    return comp;

}

vector<float>  _compareMethods(SeqTestInfo &t1,SeqTestInfo &t2)
{
    comparison comp;
    comp.seq_name=t1.seq_name;
    comp.cam=t1.cam;
    comp.method1=t1.method;
    comp.method2=t2.method;
    //analyze the whole sequence individually, and then, take the errors only in the common frames
    auto t1res=_analyzeTest(t1);
    auto t2res=_analyzeTest(t2);


//    cout<<t1res.ATE<<" "<<t1res.perctFramesTracked<<" | ";
//    cout<<t2res.ATE<<" "<<t2res.perctFramesTracked<<" | "<<std::endl;

    //intersect the frames tracked to recompute the ate with them
    for(auto it=t1res.matchedFrames.begin();it!=t1res.matchedFrames.end();){
        if (t2res.isFrame(it->frame)) it++;
        else it=t1res.matchedFrames.erase(it);
    }

    for(auto it=t2res.matchedFrames.begin();it!=t2res.matchedFrames.end();){
        if (t1res.isFrame(it->frame)) it++;
        else it=t2res.matchedFrames.erase(it);
    }

    if( t1res.matchedFrames.size()>0){

//    ucoslam::logtools::alignAndScaleToGroundTruth(t1res.matchedFrames );
    for(auto &p:t1res.matchedFrames )
        p.error=cv::norm(p.first-p.second);
//    ucoslam::logtools::alignAndScaleToGroundTruth(t2res.matchedFrames );
    for(auto &p:t2res.matchedFrames )
        p.error=cv::norm(p.first-p.second);



    //recompute ates
    double err=0;

    for(const auto &m:t1res.matchedFrames)
        err+=m.error;
    t1res.ATE=err/double(t1res.matchedFrames.size());
    err=0;
    for(const auto &m:t2res.matchedFrames)
        err+=m.error;
    t2res.ATE=err/double(t2res.matchedFrames.size());

    comp.ate1=t1res.ATE;
    comp.ate2=t2res.ATE;
    comp.track1=t1res.perctFramesTracked;
    comp.track2=t2res.perctFramesTracked;
     }
    else{
        t1res.ATE=0;
        t2res.ATE=0;
//        t1res.perctFramesTracked=0;
//        t2res.perctFramesTracked=0;
    }

    //    cout<<t1res.ATE<<" "<<t2res.ATE<<endl;
    vector<float> res(4);
    res[0]=t1res.ATE;
    res[1]=t2res.ATE;
    res[2]=t1res.perctFramesTracked;
    res[3]=t2res.perctFramesTracked;
    return res;

}

struct winnerData{
    float winA_ATE=0,winB_ATE=0;
    float winA_Tracked=0,winB_Tracked=0;
    float scoreA=0,scoreB=0;
};

winnerData winner2(const vector<comparison> &data,float p){
    winnerData wdata;

    for(const auto &v:data){
        int wA_ATE=0,wB_ATE=0,wA_T=0,wB_T=0;
        float diff=fabs(v.ate1-v.ate2);
        if ((diff> p* std::min(v.ate1,v.ate2))){
          if( v.ate1< v.ate2)
              wA_ATE=1;
            else
              wB_ATE=1;
         }

        diff=fabs(v.track1-v.track2);
        if (diff> p* std::max(v.track1,v.track2)){
            if( v.track1 > v.track2)
                wA_T=1;
            else
                wB_T=1;
        }

        wdata.winA_ATE+=wA_ATE;
        wdata.winB_ATE+=wB_ATE;
        wdata.winB_Tracked+=wB_T;
        wdata.winA_Tracked+=wA_T;

        float scoreA=0,scoreB=0;
        if( wA_ATE && wA_T)  scoreA+=1;
        if( wA_ATE && wA_T==wB_T )  scoreA+=0.5;
        if( wA_ATE==wB_ATE && wA_T)  scoreA+=0.5;

        if( wB_ATE && wB_T)  scoreB+=1;
        if( wB_ATE && wA_T==wB_T)  scoreB+=0.5;
        if( wA_ATE==wB_ATE && wB_T) scoreB+=0.5;

        wdata.scoreA+=scoreA;
        wdata.scoreB+=scoreB;


//        cout<<v.seq_name<<" "<<v.cam<<" "<<v.method1<<" "<<v.method2<< " |"<<wA_ATE<<":"<<wB_ATE<< "-"<<wA_T<<":"<<wB_T << "|"<<scoreA<<":"<<scoreB  <<"| ATE= "<<v.ate1<<" TRACK="<<v.track1<<" ATE="<<v.ate2<<
//              " TRACK="<<v.track2<<"|"<<fabs(v.ate1-v.ate2)<<">"<<p* std::min(v.ate1,v.ate2)<<"|"<<diff<<">"<<p* std::max(v.track1,v.track2)<<endl;


    }
    wdata.winA_Tracked/=double(data.size());;
    wdata.winB_Tracked/=double(data.size());;
    wdata.winA_ATE/=double(data.size());;
    wdata.winB_ATE/=double(data.size());;
    return wdata;
}

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

int main(int argc,char **argv){
    try{
        CmdLineParser cml(argc,argv);
        if (argc<3 || cml["-h"])throw std::runtime_error(" Usage: ResultsDir DataDir  [-p <float>. Default 0.01] ");

        std::vector<float> pvalues;
        if(cml["-p"])
            pvalues.push_back(stof(cml("-p","0.01")));
        else
            pvalues={0.01,0.05,0.1,0.25};


        string resutsBaseDir=argv[1];
        string dataBaseDir=argv[2];
        std::vector<string> datasets;
        std::vector<string> methods;

        for(auto ds:std::filesystem::directory_iterator(resutsBaseDir))
            if(ds.is_directory())
                datasets.push_back(ds.path().filename());


        for(auto ds:datasets)
        {
            std::filesystem::path dsPath(resutsBaseDir);
            dsPath.append(ds);

            for(auto newMethod:std::filesystem::directory_iterator(dsPath))
            {
                if(!newMethod.is_directory()) continue;

                string newM=newMethod.path().filename();

                bool exist=false;
                for(auto m:methods)
                    if(m == newM) exist=true;

                if(!exist) methods.push_back(newM);
            }
        }

//        for(auto m:methods)
//            std::cout << m << std::endl;
//        for(auto m:datasets)
//            std::cout << m << std::endl;





        if (methods.size()<=1)throw std::runtime_error("Only one method. No comparison possible.");
        for(int m=0;m<methods.size()-1;m++)
        {
            for(int m2=m+1;m2<methods.size();m2++){
                std::cout << methods[m] <<","<< methods[m2] << std::endl;

                std::map<string, std::map<string,SeqTestInfo> > dset_test_infoA,dset_test_infoB;
                std::set<string> allTests;
                for(auto dset:datasets){

                    auto allTestsA = _getDataSetTestInfo(resutsBaseDir+"/"+dset+"/"+methods[m]+"/slam",dataBaseDir+"/"+dset);
                    for(auto &ta:allTestsA){
                        dset_test_infoA[dset][ta.getDesc()]=ta;
                        allTests.insert(ta.getDesc());
                    }

                    auto allTestsB = _getDataSetTestInfo(resutsBaseDir+"/"+dset+"/"+methods[m2]+"/slam",dataBaseDir+"/"+dset);
                    for(auto &ta:allTestsB){
                        dset_test_infoB[dset][ta.getDesc()]=ta;
                        allTests.insert(ta.getDesc());
                    }
                }


                vector< vector< float> >  Results;
                vector< comparison >  Results2;
                for(auto dset:datasets){
                    for(auto test:allTests){
                        if ( dset_test_infoA[dset].count(test) &&  dset_test_infoB[dset].count(test) ){
                            // cout<<dset<<endl;
//                            Results.push_back(_compareMethods( dset_test_infoA[dset].at(test), dset_test_infoB[dset].at(test)));
                            Results2.push_back(_compareMethods2( dset_test_infoA[dset].at(test), dset_test_infoB[dset].at(test)));
                        }
                    }
                }
                // cout<<"W="<<wilcoxonSignedRankTest(Results)<<endl;
                //                auto res=winner(Results,pvalue);

                for(auto pvalue:pvalues)
                {
                    std::cout << "p-value:" <<pvalue<<std::endl;
                    auto res=winner2(Results2,pvalue);
                    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
                    cout<<"@@@@@ " <<methods[m]<<" "<<methods[m2]<<"  :Score A-B ="<<double(res.scoreA-res.scoreB)/double(Results2.size())<<endl;
                    cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
                }
            }
        }

    }catch(std::exception &ex){
    cerr<<ex.what()<<endl;
    return -1;
}
}
