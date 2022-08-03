#include "stereo_calibration.h"
#include <opencv2/highgui.hpp>
#include "levmarq.h"


double reprj_error( const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const aruco::CameraParameters &imp,const cv::Mat &rt44){
    std::vector<cv::Point2f> prepj;
     cv::Mat rv,tv;
    tslam::getRTfromMatrix44(rt44,rv,tv);
    cv::projectPoints(objPoints,rv,tv,imp.CameraMatrix,imp.Distorsion,prepj);
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<prepj.size();i++){
        if ( !std::isnan(objPoints[i].x)){
             sum+= cv::norm( points2d[i]-prepj[i]);
              nvalid++;
        }
    }
    return sum/double(nvalid);
}

void __getMarker2d_3d(vector<cv::Point2f>& p2d, vector<cv::Point3f>& p3d, const std::vector<aruco::Marker>& markers_detected,
                    const aruco::MarkerMap& bc)
{
    p2d.clear();
    p3d.clear();

    // for each detected marker
    for (size_t i = 0; i < markers_detected.size(); i++)
    {
        // find it in the bc
        auto fidx = std::string::npos;
        for (size_t j = 0; j < bc.size() && fidx == std::string::npos; j++)
            if (bc[j].id == markers_detected[i].id)
                fidx = j;
        if (fidx != std::string::npos)
        {
            for (int j = 0; j < 4; j++)
            {
                p2d.push_back(markers_detected[i][j]);
                p3d.push_back(bc[fidx][j]);
            }
        }
    }
}

template <typename T>
double __stereo_solve(const std::vector<std::vector<std::vector<cv::Point3f>>>& calib3d,
                      const std::vector<std::vector<std::vector<cv::Point2f>>>& calib2d,
                      const std::vector<std::vector<cv::Mat>>& cameraPose,
                      std::vector<aruco::CameraParameters>& camParams,
                      cv::Mat& r_io, cv::Mat& t_io)
{
    assert(r_io.type() == CV_32F);
    assert(t_io.type() == CV_32F);
    assert(t_io.total() == r_io.total());
    assert(t_io.total() == 3);

    for(auto &cp:camParams)
    {
        if ( cp.Distorsion.type() != CV_32F )
            cp.Distorsion.convertTo(cp.Distorsion,CV_32F);
        if ( cp.CameraMatrix.type() != CV_32F )
            cp.CameraMatrix.convertTo(cp.CameraMatrix,CV_32F);
    }


    auto toSol = [](const cv::Mat& r, const cv::Mat& t, const std::vector<aruco::CameraParameters> cp) {

        typename LevMarq<T>::eVector sol(24);

        for (int i = 0; i < 3; i++)
        {
            sol(i) = r.ptr<float>(0)[i];
            sol(i + 3) = t.ptr<float>(0)[i];
        }

        for(size_t i=0; i<cp.size(); i++){
            sol(6 + i*4) = cp[i].CameraMatrix.ptr<float>(0)[0]; //fx
            sol(7 + i*4) = cp[i].CameraMatrix.ptr<float>(0)[2]; //cx
            sol(8 + i*4) = cp[i].CameraMatrix.ptr<float>(1)[1]; //fy
            sol(9 + i*4) = cp[i].CameraMatrix.ptr<float>(1)[2]; //cy
        }

        for(size_t i=0; i<cp.size(); i++){
            sol(14 + i*4) = cp[i].Distorsion.ptr<float>(0)[0]; //k1
            sol(15 + i*4) = cp[i].Distorsion.ptr<float>(0)[1]; //k2
            sol(16 + i*4) = cp[i].Distorsion.ptr<float>(0)[2]; //p1
            sol(17 + i*4) = cp[i].Distorsion.ptr<float>(0)[3]; //p2
            sol(18 + i*4) = cp[i].Distorsion.ptr<float>(0)[4]; //p3
        }
        return sol;
    };
    auto fromSol = [](const typename LevMarq<T>::eVector& sol, cv::Mat& r, cv::Mat& t, std::vector<aruco::CameraParameters>& _camParams) {
        r.create(1, 3, CV_32F);
        t.create(1, 3, CV_32F);
        for (int i = 0; i < 3; i++)
        {
            r.ptr<float>(0)[i] = sol(i);
            t.ptr<float>(0)[i] = sol(i + 3);
        }

        for(size_t i=0; i<2; i++){
            _camParams[i].CameraMatrix.ptr<float>(0)[0] = sol(6 + i*4); //fx
            _camParams[i].CameraMatrix.ptr<float>(0)[1] = 0;
            _camParams[i].CameraMatrix.ptr<float>(0)[2] = sol(7 + i*4); //cx
            _camParams[i].CameraMatrix.ptr<float>(1)[0] = 0;
            _camParams[i].CameraMatrix.ptr<float>(1)[1] = sol(8 + i*4); //fy
            _camParams[i].CameraMatrix.ptr<float>(1)[2] = sol(9 + i*4); //cy
            _camParams[i].CameraMatrix.ptr<float>(2)[0] = 0;
            _camParams[i].CameraMatrix.ptr<float>(2)[1] = 0;
            _camParams[i].CameraMatrix.ptr<float>(2)[2] = 1;

            _camParams[i].Distorsion.ptr<float>(0)[0] = sol(14 + i*4); //k1
            _camParams[i].Distorsion.ptr<float>(0)[1] = sol(15 + i*4); //k2
            _camParams[i].Distorsion.ptr<float>(0)[2] = sol(16 + i*4); //p1
            _camParams[i].Distorsion.ptr<float>(0)[3] = sol(17 + i*4); //p2
            _camParams[i].Distorsion.ptr<float>(0)[4] = sol(18 + i*4); //p3
        }
    };

    auto err_f = [&](const typename LevMarq<T>::eVector& sol, typename LevMarq<T>::eVector& err) {

        std::vector<cv::Point2f> p2d_rej;
        cv::Mat r, t;

        fromSol(sol, r, t, camParams);

        cv::Mat X = tslam::getRTMatrix(r, t);

        int el=0;
        for(uint imgId=0; imgId<cameraPose[0].size(); imgId++)
            el += calib3d[1][imgId].size() + calib3d[0][imgId].size();
        err.resize(el * 4);

        int err_idx = 0;
        for(uint imgId=0; imgId<cameraPose[0].size(); imgId++)
        {
            std::vector<cv::Mat> vr;
            std::vector<cv::Mat> vt;
            vr.resize(2);
            vt.resize(2);
            tslam::getRTfromMatrix44(X*cameraPose[1][imgId], vr[0], vt[0]);
            tslam::getRTfromMatrix44(X.inv()*cameraPose[0][imgId], vr[1], vt[1]);


            //cross error using extrinsic
            for(size_t c=0; c<camParams.size(); c++)
            {
                cv::projectPoints(calib3d[c][imgId], vr[c], vt[c], camParams[c].CameraMatrix, camParams[c].Distorsion, p2d_rej);
                for (size_t i = 0; i < p2d_rej.size(); i++)
                {
                    err(err_idx++) = p2d_rej[i].x - calib2d[c][imgId][i].x;
                    err(err_idx++) = p2d_rej[i].y - calib2d[c][imgId][i].y;
                }
            }


            //error each camera
            for(size_t c=0; c<camParams.size(); c++)
            {
                cv::Mat r,t;
                tslam::getRTfromMatrix44(cameraPose[c][imgId], r, t);
                cv::projectPoints(calib3d[c][imgId], r, t, camParams[c].CameraMatrix, camParams[c].Distorsion, p2d_rej);
                for (size_t i = 0; i < p2d_rej.size(); i++)
                {
                    err(err_idx++) = p2d_rej[i].x - calib2d[c][imgId][i].x;
                    err(err_idx++) = p2d_rej[i].y - calib2d[c][imgId][i].y;
                }
            }
        }
     };

    LevMarq<T> solver;
    solver.setParams(100, 0.01, 0.01);
    solver.verbose()=true;
    typename LevMarq<T>::eVector sol = toSol(r_io, t_io, camParams);
    auto err = solver.solve(sol, err_f);

    fromSol(sol, r_io, t_io, camParams);
    return err;
}

StereoCalibration::StereoCalibration()
{
}

StereoCalibration::~StereoCalibration()
{
}

void StereoCalibration::setParams(const float markerSize, const aruco::MarkerMap& markerMap)
{
    _msize = markerSize;
    _mmap = markerMap;

    _mdetector.setDictionary( _mmap.getDictionary());
    _mdetector.setDetectionMode(aruco::DM_NORMAL);
}

void StereoCalibration::calibration(const std::vector<std::string>& imagelist, bool displayDetectedMarkers)
{
    if( imagelist.size() % 2 != 0 )
    {
        std::cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    int ncameras = 2;
    int nimages = (int)imagelist.size()/ncameras;

    _camParams.resize(ncameras);

    std::vector<std::vector<std::vector<aruco::Marker>>> detected_markers;
    detected_markers.resize(ncameras);
    for(int c=0; c<ncameras; c++)
        detected_markers[c].resize(nimages);

    std::vector<vector<std::vector<cv::Point3f>>> calib_p3d;
    std::vector<vector<std::vector<cv::Point2f>>> calib_p2d;
    calib_p3d.resize(ncameras);
    calib_p2d.resize(ncameras);

    int i, j, k;
    for(i = 0; i < nimages; i++ )
    {
        for(k = 0; k < 2; k++ )
        {
            const std::string& filename = imagelist[i*2+k];
            std::cerr<<"Processing "<<filename<<std::endl;
            cv::Mat img = cv::imread(filename, 0);
            if(img.empty())
                continue;

            _camParams[k].CamSize = img.size();
            detected_markers[k][i] = _mdetector.detect(img);

            if(displayDetectedMarkers)
            {
                cv::Mat imgCopy;
                cv::cvtColor(img,imgCopy,cv::COLOR_GRAY2BGR);
                for (auto dm : detected_markers[k][i])
                    dm.draw(imgCopy, cv::Scalar(0, 0, 255),
                            static_cast<int>( max(float(1.f), 1.5f * float(imgCopy.cols) / 1000.f)));

                // show input with augmented information and  the thresholded image
                cv::imshow("in", imgCopy);
                // write to video if required
                cv::waitKey(100);  // wait for key to be pressed
            }
        }
    }

    for(int c=0; c<ncameras; c++)
    {
        for(auto markers:detected_markers[c])
        {
            vector<cv::Point2f> p2d;
            vector<cv::Point3f> p3d;
            __getMarker2d_3d(p2d, p3d, markers, _mmap);
            if (p3d.size() > 0)
            {
                calib_p2d[c].push_back(p2d);
                calib_p3d[c].push_back(p3d);
            }
        }

        cout << "Starting calibration cam"<< c << endl;
        vector<cv::Mat> vr, vt;
        cv::calibrateCamera(calib_p3d[c], calib_p2d[c], _camParams[c].CamSize, _camParams[c].CameraMatrix, _camParams[c].Distorsion, vr, vt);

        //compute the average reprojection error
        std::pair<double,int> sum={0,0};
        for(size_t v=0;v<calib_p2d[c].size();v++){
            vector<cv::Point2f> repj;
            cv::projectPoints(calib_p3d[c][v],vr[v],vt[v],_camParams[c].CameraMatrix, _camParams[c].Distorsion, repj);
            for(size_t p=0;p<calib_p3d[c][v].size();p++){
                sum.first+=cv::norm(repj[p]-calib_p2d[c][v][p]);
                sum.second++;
            }
        }
        cerr << "repj error cam"+to_string(c)+"=" << sum.first/double(sum.second) << endl;
    }


    cout << "Starting stereo calibration" << endl;

    std::vector<aruco::MarkerMapPoseTracker> MSPoseTrackers;
    MSPoseTrackers.resize(ncameras);
    for(k=0; k<ncameras; k++)
        MSPoseTrackers[k].setParams(_camParams[k], _mmap, _msize);


    std::vector<std::vector<cv::Mat>> cameraPose;
    cameraPose.resize(ncameras);
    for(auto &cp:cameraPose)
        cp.resize(nimages);

    for(i = j = 0; i < nimages; i++ )
    {
        cerr << "Camera poses img"<<i << endl;

        for(k = 0; k < ncameras; k++ )
        {
            MSPoseTrackers[k].reset();
            if(MSPoseTrackers[k].estimatePose(detected_markers[k][i]))
                cameraPose[k][j] = MSPoseTrackers[k].getRTMatrix();
            else
                break;

            vector<cv::Point2f> p2d;
            vector<cv::Point3f> p3d;
            __getMarker2d_3d(p2d, p3d, detected_markers[k][i], _mmap);
            if (p3d.size() > 0)
            {
                calib_p2d[k][j] = p2d;
                calib_p3d[k][j] = p3d;
            }
        }
        if(k==ncameras)
            j++;
    }

    for(k=0; k<ncameras; k++)
    {
        cameraPose[k].resize(j);
        calib_p2d[k].resize(j);
        calib_p3d[k].resize(j);
    }

    stereo_solve(calib_p3d, calib_p2d, cameraPose, _rvec, _tvec);
}

double StereoCalibration::stereo_solve(const std::vector<std::vector<std::vector<cv::Point3f>>>& calib3d,
                    const std::vector<std::vector<std::vector<cv::Point2f>>>& calib2d,
                    const std::vector<std::vector<cv::Mat>>& cameraPose,
                    cv::Mat& r_io, cv::Mat& t_io)
{

    float minErr=std::numeric_limits<float>::max(), minErrId=-1;

    int ncameras=cameraPose.size();
    int nimages=cameraPose[0].size();

    //Get pose estimation with lower error
    for(int i=0; i<nimages; i++)
    {
        float err=0.f;
        for(int k=0; k<ncameras; k++)
            err+=reprj_error( calib3d[k][i], calib2d[k][i], _camParams[k], cameraPose[k][i]);
        err/= ncameras;

        tslam::getRTfromMatrix44(cameraPose[0][i]*cameraPose[1][i].inv(), r_io, t_io);

        if(err < minErr)
        {
            minErr = err;
            minErrId = i;
        }
    }

    //initial rvec, tvec
    tslam::getRTfromMatrix44(cameraPose[0][minErrId]*cameraPose[1][minErrId].inv(), r_io, t_io);
    //stero calibration
    __stereo_solve<double>(calib3d, calib2d, cameraPose, _camParams, r_io, t_io);

    //Show reprj error using new intrinsic params
    for(int i=0; i<nimages; i++)
    {
        //Prj error individual cameras
        float err=0.f;
        for(int k=0; k<ncameras; k++)
            err+=reprj_error( calib3d[k][i], calib2d[k][i], _camParams[k], cameraPose[k][i]);
        err/= ncameras;


        //Cross error
        std::vector<cv::Mat> Transf;
        Transf.resize(2);
        Transf[0] = tslam::getRTMatrix(r_io, t_io)*cameraPose[1][i];
        Transf[1] = tslam::getRTMatrix(r_io, t_io).inv()*cameraPose[0][i];
        float err2=0.f;
        for(int k=0; k<ncameras; k++)
            err2+=reprj_error( calib3d[k][i], calib2d[k][i], _camParams[k], Transf[k]);
        err2/= ncameras;

        std::cout <<"Image("<<i<<"/"<<nimages-1<<") Avrg prj error:"<< err<< ", Avrg cross error:"<< err2<< std::endl;
    }
}







