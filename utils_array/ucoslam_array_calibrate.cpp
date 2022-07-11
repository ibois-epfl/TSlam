#include <aruco/aruco.h>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include "stereo_calibration.h"
#include "aruco_calibration_grid_board_a4.h"

#include "dirreader.h"


using namespace std;
using namespace cv;
using namespace aruco;

float TheMarkerSize = -1;
string TheMarkerMapConfigFile;
aruco::MarkerMap TheMarkerMap;
StereoCalibration TheCalibrator;

vector<string> ReadImages(string dirPath);

class CmdLineParser
{  int argc;char** argv;public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    bool operator[](string param)    {int idx = -1;   for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;return (idx != -1); }
    string operator()(string param, string defvalue = "-1"){int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);}
};

/************************************
 *
 *
 ************************************/

int main(int argc, char** argv)
{
    try
    {
        string imageDir;

        CmdLineParser cml(argc, argv);
        if (argc < 3 || cml["-h"])
        {
            cout << "Usage: imageDir  out_stereo_calibration.yml [-m markermapConfig.yml (configuration of the "
                    "board. If use default one (in utils), no need to set this)]    [-size <float> :(value in meters "
                    "of a marker. If you provide a board that contains that information, this is ommited) ] "
                 << endl;
            cout <<"Image must all be in the same directory with naming convention: cam0_0.jpg cam1_0.jpg cam0_1.jpg .... right1_10.jpg"<<endl;

            return -1;
        }

        TheMarkerSize = stof(cml("-size","1"));
        TheMarkerMapConfigFile=cml("-m","");

        imageDir = argv[1];
        if(imageDir.empty()){
            cout<<"Image must all be in the same directory with naming convention: cam0_0.jpg cam1_0.jpg cam0_10.jpg .... cam1_10.jpg"<<endl;
            return -1;
        }

        auto imagelist=ReadImages(imageDir);
        for(auto fn:imagelist) cout<<fn <<endl;


        // load board info
        if (TheMarkerMapConfigFile.empty())
        {
            stringstream sstr;
            sstr.write((char*)default_a4_board, default_a4_board_size);
            TheMarkerMap.fromStream(sstr);
        }
        else
            TheMarkerMap.readFromFile(TheMarkerMapConfigFile);

        // is in meters
        if (!TheMarkerMap.isExpressedInMeters())
        {
            if (TheMarkerSize == -1)
            {
                cerr << "Need to specify the length of the board with -size" << endl;
                return -1;
            }
            else
                TheMarkerMap = TheMarkerMap.convertToMeters(TheMarkerSize);
        }

        TheCalibrator.setParams(TheMarkerSize, TheMarkerMap);
        TheCalibrator.calibration(imagelist, true);
        cv::Mat extrinsic = TheCalibrator.getRTMatrix();

        cv::FileStorage fs;
        cerr<<"Saving to: "<<argv[2]<<endl;
        fs.open(argv[2], FileStorage::WRITE);
        if( fs.isOpened() )
        {
            fs << "array_size" << 2;
            fs << "img_width_cam0"<<TheCalibrator._camParams[0].CamSize.width;
            fs << "img_height_cam0"<<TheCalibrator._camParams[0].CamSize.height;
            fs << "img_width_cam1"<<TheCalibrator._camParams[1].CamSize.width;
            fs << "img_height_cam1"<<TheCalibrator._camParams[1].CamSize.height;

            fs << "M_cam0" << TheCalibrator._camParams[0].CameraMatrix  << "D_cam0" << TheCalibrator._camParams[0].Distorsion;
            fs << "M_cam1" << TheCalibrator._camParams[1].CameraMatrix  << "D_cam1" << TheCalibrator._camParams[1].Distorsion;
            fs << "R_cam0_cam1" << TheCalibrator.getRvec()              << "T_cam0_cam1" << TheCalibrator.getTvec();
            fs.release();
        }
        else
            cerr << "Error: can not save the extrinsic parameters\n";

        return 0;
    }
    catch (std::exception& ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}


vector< string  > ReadImages(string dirPath)
{
    auto getFileName=[](const std::string &path){
        auto pos=path.rfind("/");
        if(pos==std::string::npos) return path;
        string name;
        for(size_t i=pos+1;i<path.size();i++)
            name.push_back(path[i]);
        return name;

    };
    auto parseName=[&](const std::string &path){

        string name=getFileName(path);
        auto pos=name.rfind("cam0_");
        int leftright=-1;

        if(pos!=std::string::npos)  {
            leftright=0;
            pos+=5;
        }
        else{
            pos=name.rfind("cam1_");
            if(pos==std::string::npos) return std::make_pair(string(""),-1);
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
        return  std::make_pair(name, std::stoi(number));
    };

    DirReader Dir;
    auto files=Dir.read(dirPath,".jpg .ppm .png .pgm .bmp",DirReader::Params(true));

    //now, find left ritgh names
    std::map<int,std::pair<string,string> > id_leftRight;

    for(auto file:files){
            auto name_number=parseName(file);
            if( name_number.second!=-1){
                if( name_number.first.find("cam0_")!=std::string::npos)
                    id_leftRight[name_number.second].first=file;
                else if(file.find("cam1_")!=std::string::npos)
                        id_leftRight[name_number.second].second=file;
                else throw  std::runtime_error("Error in cam0 cam1 name");
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
