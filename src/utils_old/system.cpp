#include <list>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/markermap.h>
#include "utils/system.h"
#include "basictypes/misc.h"
#include "basictypes/debug.h"
#include "basictypes/timers.h"
#include "optimization/pnpsolver.h"
#include "optimization/globaloptimizer.h"
#include "optimization/ippe.h"
#include "basictypes/io_utils.h"
#include "map_types/keyframedatabase.h"
#include "utils/mapinitializer.h"
#include "utils/mapmanager.h"
#include "map.h"
#include "basictypes/se3.h"
#include "basictypes/osadapter.h"
#include "map_types/covisgraph.h"
#include "utils/frameextractor.h"
#include "basictypes/hash.h"

namespace

tslam {

    Params System::sysParams;

    Params &System::getParams() { return sysParams; }

    uint32_t System::getCurrentKeyFrameIndex() { return currentKeyFrameId_105767; }

    std::shared_ptr<Map> System::getMap() { return TheMap; }

    System::System() {
        sysFrameExtractor = std::make_shared<FrameExtractor>();
        sysMapIntializer = std::make_shared<MapInitializer>();
        sysMapManager = std::make_shared<MapManager>();
        _1320287184975591154 = std::make_shared<tslam::STagDetector>();
    }

    System::~System() { waitForFinished(); }

    void System::_14789688456123595594() {
        sysParams.nthreads_feature_detector = max(1, sysParams.nthreads_feature_detector);
        std::shared_ptr<Feature2DSerializable> feature2DSerializable = Feature2DSerializable:: create(sysParams.kpDescriptorType);
        feature2DSerializable->setParams(sysParams.extraParams);
        sysParams.maxDescDistance = feature2DSerializable->getMinDescDistance();

        sysFrameExtractor->setParams(feature2DSerializable, sysParams, _1320287184975591154);
        sysFrameExtractor->removeFromMarkers() = sysParams.removeKeyPointsIntoMarkers;
        sysFrameExtractor->detectMarkers() = sysParams.detectMarkers;
        sysFrameExtractor->detectKeyPoints() = sysParams.detectKeyPoints;
    }

    void System::setParams(std::shared_ptr<Map> _11093822290287, const Params &_2654435881,
                           const string &_4953871428288621283, std::

                           shared_ptr<

            tslam::

            MarkerDetector>

                           _1516358670470627782) {

        TheMap =

                _11093822290287;

        sysParams =

                _2654435881;

        _1320287184975591154 =

                _1516358670470627782;

        if (

                !

                        _1320287184975591154)

            _1320287184975591154 =
                    std::

                    make_shared<
                            STagDetector>
                            (
                                    sysParams);

        _14789688456123595594();

        if

                (

                TheMap->

                        isEmpty()

                ) {

            trackingState =

                    STATE_LOST;

            if

                    (!
                    _4953871428288621283.empty()
                    ) {

                TheMap->
                        TheKFDataBase.loadFromFile(

                        _4953871428288621283);

            }

            MapInitializer::
            Params _3005399798454910266;

            if

                    (
                    sysParams.forceInitializationFromMarkers)

                _3005399798454910266.mode =

                        MapInitializer::
                        ARUCO;

            else
                _3005399798454910266.mode =

                        MapInitializer::

                        BOTH;

            _3005399798454910266.minDistance =

                    sysParams.minBaseLine;

            _3005399798454910266.markerSize =

                    sysParams.aruco_markerSize;

            _3005399798454910266.aruco_minerrratio_valid =

                    sysParams.aruco_minerrratio_valid;

            _3005399798454910266.allowArucoOneFrame =

                    sysParams.aruco_allowOneFrameInitialization;

            _3005399798454910266.max_makr_rep_err =

                    2.5;

            _3005399798454910266.minDescDistance =

                    sysParams.maxDescDistance;

            sysMapIntializer->

                    setParams(

                    _3005399798454910266);

        } else
            trackingState =
                    STATE_LOST;

    }

    void

    System::

    waitForFinished() {

        sysMapManager->

                stop();

        sysMapManager->
                mapUpdate();

        if (

                sysMapManager->bigChange()

                ) {

            curFrame.pose_f2g =

                    sysMapManager->

                            getLastAddedKFPose();

            curPose_ns =

                    curFrame.pose_f2g;

        }

    }

    void System::resetTracker() {
        waitForFinished();
        currentKeyFrameId_105767 = -1;
        curPose_ns = se3();
        trackingState = STATE_LOST;
        curFrame.clear();
        lastFrame.clear();
        _14463320619150402643 = cv::Mat();
        _10558050725520398793 = -1;
    }

    cv::Mat System::process(const Frame &inputFrame) {

        se3 _16937225862434286412 = curPose_ns;

        if ((void *) &inputFrame != (void *) &curFrame) {
            swap(lastFrame, curFrame);
            curFrame = inputFrame;
        }

        if (sysMode == MODE_SLAM && !sysMapManager->hasMap())
            sysMapManager->setParams(TheMap, sysParams.enableLoopClosure);

        if (!sysParams.runSequential && sysMode == MODE_SLAM)
            sysMapManager->start();

        for (auto &_175247760135: lastFrame.ids)
            if (_175247760135 != std::numeric_limits<uint32_t>::max()) {
                if (!TheMap->map_points.is(_175247760135))
                    _175247760135 = std::numeric_limits<uint32_t>::max();
                else if (TheMap->map_points[_175247760135].isBad())
                    _175247760135 = std::numeric_limits<uint32_t>::max();
            }

        if (TheMap->isEmpty() && sysMode == MODE_SLAM) {
            if (_2016327979059285019(curFrame))
                trackingState = STATE_TRACKING;
        } else {
            if (trackingState == STATE_TRACKING) {
                currentKeyFrameId_105767 = getRefKeyFrameId(lastFrame, curPose_ns);
                curPose_ns = _11166622111371682966(curFrame, curPose_ns); // bug here

                if (!curPose_ns.isValid())
                    trackingState = STATE_LOST;
            }

            if (trackingState == STATE_LOST) {
                se3 _5564636146947005941;
                if (_16487919888509808279(curFrame, _5564636146947005941)) {
                    trackingState = STATE_TRACKING;
                    curPose_ns = _5564636146947005941;
                    currentKeyFrameId_105767 = getRefKeyFrameId(curFrame, curPose_ns);
                    _10558050725520398793 = curFrame.fseq_idx;
                }
            }

            if (trackingState == STATE_TRACKING) {
                curFrame.pose_f2g = curPose_ns;
                if (sysMode == MODE_SLAM &&
                    ((curFrame.fseq_idx >= _10558050725520398793 + 5) || (_10558050725520398793 == -1)))
                    sysMapManager->newFrame(curFrame, currentKeyFrameId_105767);
            }
        }

        if (trackingState == STATE_LOST && sysMode == MODE_SLAM && TheMap->keyframes.size() <= 5 &&
            TheMap->keyframes.size() != 0) {
            sysMapManager->reset();
            TheMap->clear();
            sysMapIntializer->reset();
            sysMapManager->setParams(TheMap, sysParams.enableLoopClosure);
        }

        if (trackingState == STATE_TRACKING) {
            _14463320619150402643 = cv::Mat::eye(4, 4, CV_32F);
            if (_16937225862434286412.isValid()) {
                _14463320619150402643 = curPose_ns.convert() * _16937225862434286412.convert().inv();
            }
        } else {
            _14463320619150402643 = cv::Mat();
        }
        curFrame.pose_f2g = curPose_ns;

        // if (++_13033649816026327368 > (10 * 4 * 12 * 34 * 6) / 2) curPose_ns = cv::Mat();

        if (trackingState == STATE_LOST) return cv::Mat();
        else return curPose_ns;
    }

cv::Mat System::process(cv::Mat &in_image, const ImageParams &imgParams,
                        uint32_t _9933887380370137445, const cv::Mat &_46082575014988268,
                        const cv::Mat &_1705635550657133790) {
    swap(lastFrame, curFrame);
     std::thread mapUpdateThread;
    if (sysMode == MODE_SLAM) {
         mapUpdateThread = std::thread([&]() {
            if (sysMapManager->mapUpdate()) {
                if (sysMapManager->bigChange()) {
                    curFrame.pose_f2g = sysMapManager->getLastAddedKFPose();
                    curPose_ns = sysMapManager->getLastAddedKFPose();
                }
            };
         });
    }

    if (_46082575014988268.empty() && _1705635550657133790.empty())
        sysFrameExtractor->process(in_image, imgParams, curFrame, _9933887380370137445);
    else if (_1705635550657133790.empty())
        sysFrameExtractor->process_rgbd(in_image, _46082575014988268, imgParams,curFrame, _9933887380370137445);
    else
        sysFrameExtractor->processStereo(in_image, _1705635550657133790, imgParams,curFrame, _9933887380370137445);

    if (sysParams.autoAdjustKpSensitivity) {
        int _1699599737904718822 = sysParams.maxFeatures - curFrame.und_kpts.size();
        if (_1699599737904718822 > 0) {
            float _46082575832048655 = 1.0f - (float(_1699599737904718822) / float(curFrame.und_kpts.size()));
            float _6148074839757474704 = sysFrameExtractor->getSensitivity() + _46082575832048655;
            _6148074839757474704 = std::max(_6148074839757474704, 1.0f);
            sysFrameExtractor->setSensitivity(_6148074839757474704);
        } else {
            sysFrameExtractor->setSensitivity(sysFrameExtractor->getSensitivity()* 0.95);
        }
    }

    if(sysMode == MODE_SLAM) mapUpdateThread.join();
    cv::Mat _3005399805025936106 = process(curFrame);

    float _6154865401824487276 = sqrt(float(curFrame.imageParams.CamSize.area())/ float(in_image.size().area()));

    _14031550457846423181(in_image, 1. / _6154865401824487276);

    auto _5829441678613027716 = [](const uint32_t &_11093821926013) {
        std::stringstream _706246330191125;
        _706246330191125 <<_11093821926013;
        return _706246330191125.str();
    };

    _14938529070154896274(
            in_image,
            "\x4d\x61\x70\x20\x50\x6f\x69\x6e\x74\x73\x3a" + _5829441678613027716(TheMap->map_points.size()),
            cv::Point(20, in_image.rows - 20));

    _14938529070154896274(
            in_image,
            "\x4d\x61\x70\x20\x4d\x61\x72\x6b\x65\x72\x73\x3a" + _5829441678613027716(TheMap->map_markers.size()),
            cv::Point(20, in_image.rows - 40));

    _14938529070154896274(
            in_image,
            "\x4b\x65\x79\x46\x72\x61\x6d\x65\x73\x3a" + _5829441678613027716(TheMap->keyframes.size()),
            cv::Point(20, in_image.rows - 60));

    int _16937201858692939798 = 0;

    for (auto _175247760135: curFrame.ids)
        if (_175247760135 != std::numeric_limits<uint32_t>::max())
            _16937201858692939798++;

    _14938529070154896274(
            in_image,
            "\x4d\x61\x74\x63\x68\x65\x73\x3a" + _5829441678613027716(_16937201858692939798),
            cv::Point(20, in_image.rows - 80));

    if (fabs(_6154865401824487276 - 1) > 1e-3)
        _14938529070154896274(
                in_image,
                "\x49\x6d\x67\x2e\x53\x69\x7a\x65\x3a" +
                    _5829441678613027716(curFrame.imageParams.CamSize.width) +
                    "\x78" + _5829441678613027716(curFrame.imageParams.CamSize.height),
                cv::Point(20, in_image.rows - 100));

    return _3005399805025936106;

    }

cv::Mat System::process(vector<cv::

                    Mat>

            &_3005401535270843804, ImageParams &_4702029808027735906, uint32_t

            _9933887380370137445) {

        swap(
                lastFrame, curFrame);

        std::thread mapUpdateThread;

        if

                (sysMode ==

                 MODE_SLAM)

            mapUpdateThread = std::thread([&]() {
                if (sysMapManager->mapUpdate()) {
                    if (

                                            sysMapManager->

                                                    bigChange()

                                            ) {

                                        curFrame.pose_f2g = sysMapManager->

                                                getLastAddedKFPose();
                                        curPose_ns =

                                                sysMapManager->
                                                        getLastAddedKFPose();

                                    }

                                }

                                ;

                            });

        sysFrameExtractor->

                processArray(
                _3005401535270843804, _4702029808027735906, curFrame, _9933887380370137445, sysMapIntializer);

        if (

                sysParams.autoAdjustKpSensitivity) {

            int
                    _1699599737904718822 =

                    sysParams.maxFeatures - curFrame.und_kpts.size();

            if (
                    _1699599737904718822 >

                    0) {

                float

                        _46082575832048655 =

                        1.0f - (

                                float(

                                        _1699599737904718822)
                                / float(
                                        curFrame.und_kpts.size()

                                )

                        );

                float
                        _6148074839757474704 =
                        sysFrameExtractor->

                                getSensitivity()

                        + _46082575832048655;

                _6148074839757474704 =

                        std::

                        max(_6148074839757474704, 1.0f);

                sysFrameExtractor->

                        setSensitivity(

                        _6148074839757474704);

            } else {

                sysFrameExtractor->

                        setSensitivity(

                        sysFrameExtractor->

                                getSensitivity()

                        * 0.95);

            }

        }

        if (sysMode == MODE_SLAM) mapUpdateThread.join();

        cv::

        Mat _3005399805025936106 = process(

                curFrame);

        float

                _6154865401824487276 =

                sqrt(

                        float(

                                curFrame.imageParams.CamSize.area()

                        ) / float(
                                _3005401535270843804[0]

                                        .size()

                                        .area()

                        )

                );

        _14031550457846423181(

                _3005401535270843804[

                        0], 1. / _6154865401824487276);

        auto

                _5829441678613027716 =

                [

                ]

                        (
                                const uint32_t &_11093821926013) {
                    std::

                    stringstream _706246330191125;
                    _706246330191125 <<

                                     _11093821926013;
                    return _706246330191125.str();

                };

        _14938529070154896274(

                _3005401535270843804[

                        0], "\x4d\x61\x70\x20\x50\x6f\x69\x6e\x74\x73\x3a" + _5829441678613027716(
                        TheMap->

                                map_points.size()

                ), cv::

                Point(

                        20, _3005401535270843804[

                                    0]

                                    .rows - 20)

        );

        _14938529070154896274(
                _3005401535270843804[

                        0], "\x4d\x61\x70\x20\x4d\x61\x72\x6b\x65\x72\x73\x3a" + _5829441678613027716(

                        TheMap->

                                map_markers.size()

                ), cv::

                Point(
                        20, _3005401535270843804[

                                    0]

                                    .rows - 40)

        );

        _14938529070154896274(

                _3005401535270843804[
                        0], "\x4b\x65\x79\x46\x72\x61\x6d\x65\x73\x3a" + _5829441678613027716(

                        TheMap->

                                keyframes.size()

                ), cv::Point(

                        20, _3005401535270843804[

                                    0]

                                    .rows - 60)

        );

        int
                _16937201858692939798 =

                0;
        for (

            auto _175247760135: curFrame.ids)

            if (
                    _175247760135 !=

                    std::

                    numeric_limits<

                            uint32_t>
                    ::

                    max()

                    )
                _16937201858692939798++;

        _14938529070154896274(

                _3005401535270843804[

                        0], "\x4d\x61\x74\x63\x68\x65\x73\x3a" + _5829441678613027716(

                        _16937201858692939798), cv::

                Point(
                        20, _3005401535270843804[

                                    0]
                                    .rows - 80)

        );
        if (

                fabs(

                        _6154865401824487276 - 1)

                >
                1e-3)

            _14938529070154896274(

                    _3005401535270843804[0], "\x49\x6d\x67\x2e\x53\x69\x7a\x65\x3a" + _5829441678613027716(
                            curFrame.imageParams.CamSize.width)

                                             + "\x78" + _5829441678613027716(

                            curFrame.imageParams.CamSize.height), cv::

                    Point(

                            20, _3005401535270843804[

                                        0]

                                        .rows - 100)

            );

        return

                _3005399805025936106;

    }

    void

    System::

    _14938529070154896274(
            cv::

            Mat &_175247760140, string _706246331661728, cv::

            Point _2654435881) {

        float
                _706246308256699 =

                float(
                        _175247760140.cols)

                / float(
                        1280);
        cv::

        putText(

                _175247760140, _706246331661728, _2654435881, cv::

                FONT_HERSHEY_SIMPLEX, 0.5 * _706246308256699, cv::
                Scalar(

                        0, 0, 0), 3 * _706246308256699);

        cv::

        putText(

                _175247760140, _706246331661728, _2654435881, cv::

                FONT_HERSHEY_SIMPLEX, 0.5 * _706246308256699, cv::

                Scalar(

                        125, 255, 255), 1 * _706246308256699);

    }

    string System::

    getSignatureStr()
    const {

        return

                _2102381941757963317(
                        _13507858972549420551()

                );

    }

    uint64_t System::

    _13507858972549420551(bool _46082575779493229)

    const {

        Hash _11093822380353;

        _11093822380353 += TheMap->

                getSignature(
                _46082575779493229);

        if (

                _46082575779493229)
            cerr <<
                 R"(\tSystem 1. sig=)" <<

                 _11093822380353 <<

                 endl;

        _11093822380353 +=

                sysParams.getSignature();

        if (
                _46082575779493229)

            cerr << R"(\tSystem 2. sig=)" <<
                 _11093822380353 <<
                 endl;

        for (

                int _2654435874 =

                        0;
                _2654435874 <

                6;
                _2654435874++

                )
            _11093822380353 +=

                    curPose_ns[

                            _2654435874];

        if (
                _46082575779493229)

            cerr <<

                 R"(\tSystem 3. sig=)" << _11093822380353 <<

                 endl;

        _11093822380353.add(

                currentKeyFrameId_105767);

        if (

                _46082575779493229)

            cerr <<

                 R"(\tSystem 4. sig=)" <<
                 _11093822380353 <<

                 endl;

        _11093822380353 += curFrame.getSignature();

        if (

                _46082575779493229)

            cerr <<

                 R"(\tSystem 5. sig=)" <<

                 _11093822380353 <<

                 endl;

        _11093822380353 +=

                _13028158409047949416;

        if (

                _46082575779493229)

            cerr <<

                 "\x5c\x74\x53\x79\x73\x74\x65\x6d\x20\x37\x2e\x20\x73\x69\x67\x3d" <<
                 _11093822380353 <<

                 endl;

        _11093822380353 += trackingState;

        if (

                _46082575779493229)
            cerr <<

                 "\x5c\x74\x53\x79\x73\x74\x65\x6d\x20\x38\x2e\x20\x73\x69\x67\x3d" <<
                 _11093822380353 <<
                 endl;

        _11093822380353 +=

                sysMode;

        if (

                _46082575779493229)

            cerr <<

                 "\x5c\x74\x53\x79\x73\x74\x65\x6d\x20\x39\x2e\x20\x73\x69\x67\x3d" <<
                 _11093822380353 <<

                 endl;

        _11093822380353 +=
                lastFrame.getSignature();

        if (

                _46082575779493229)

            cerr <<

                 "\x5c\x74\x53\x79\x73\x74\x65\x6d\x20\x31\x30\x2e\x73\x69\x67\x3d" <<

                 _11093822380353 << endl;

        _11093822380353 +=

                sysMapManager->

                        getSignature();

        if (_46082575779493229)

            cerr <<

                 "\x5c\x74\x53\x79\x73\x74\x65\x6d\x20\x31\x31\x2e\x73\x69\x67\x3d" <<
                 _11093822380353 <<

                 endl;

        _11093822380353 +=

                _14463320619150402643;

        _11093822380353 +=

                _10558050725520398793;

        if (

                _46082575779493229)
            cerr <<

                 "\x5c\x74\x53\x79\x73\x74\x65\x6d\x20\x31\x32\x2e\x73\x69\x67\x3d" <<

                 _11093822380353 <<
                 endl;

        return

                _11093822380353;

    }

    cv::

    Mat System::_4145838251597814913(

            const Frame &inputFrame) {

        std::
        vector<

                uint32_t> _4240939334638385660;

        vector<pair<

                cv::

                Mat, double>

        >
                _5923954032168212568;

        vector<

                cv::

                Point3f>

                _7045032207766252521;

        vector<

                cv::

                Point2f>

                _7045032207766252456;

        for (

            auto _markerObservation: inputFrame.markers) {

            if

                    (TheMap->

                    map_markers.find(_markerObservation.id)

                     !=

                     TheMap->

                             map_markers.end()
                    ) {

                tslam::
                Marker &_6807036686937475945 =

                        TheMap->

                                map_markers[

                                _markerObservation.id];

                cv::
                Mat _9983235290341257781 =
                        _6807036686937475945.pose_g2m;

                auto
                        _11093822296219 =

                        _6807036686937475945.get3DPoints();

                _7045032207766252521.insert(

                        _7045032207766252521.end(), _11093822296219.begin(), _11093822296219.end()

                );

                _7045032207766252456.insert(

                        _7045032207766252456.end(), _markerObservation.und_corners.begin(), _markerObservation.und_corners.end()

                );

                auto
                        _1515389571633683069 =

                        IPPE::
                        solvePnP(

                                sysParams.aruco_markerSize, _markerObservation.und_corners,
                                inputFrame.imageParams.CameraMatrix, inputFrame.imageParams.Distorsion);

                for (
                    auto _16937226146608628973: _1515389571633683069)

                    _5923954032168212568.push_back(

                            make_pair(

                                    _16937226146608628973 * _9983235290341257781.inv(), -1)
                    );

            }
        }

        if
                (_7045032207766252521.size()

                 ==

                 0)
            return cv::

            Mat();

        for (

            auto &_46082575779853237: _5923954032168212568) {

            vector<
                    cv::

                    Point2f>

                    _1535067723848375914;

            se3 _16937226146609453200 =

                    _46082575779853237.first;

            project(

                    _7045032207766252521, inputFrame.imageParams.CameraMatrix, _16937226146609453200.convert(),
                    _1535067723848375914);

            _46082575779853237.second =

                    0;

            for (

                    size_t _2654435874 =

                            0;
                    _2654435874 <
                    _1535067723848375914.size();

                    _2654435874++

                    )

                _46082575779853237.second +=

                        (

                                _1535067723848375914[

                                        _2654435874]

                                        .x - _7045032207766252456[

                                        _2654435874]

                                        .x)

                        * (

                                _1535067723848375914[

                                        _2654435874]
                                        .x - _7045032207766252456[

                                        _2654435874]

                                        .x)

                        + (

                                  _1535067723848375914[

                                          _2654435874]

                                          .y - _7045032207766252456[

                                          _2654435874]

                                          .y) * (

                                  _1535067723848375914[
                                          _2654435874]

                                          .y - _7045032207766252456[

                                          _2654435874]

                                          .y);

        }

        std::
        sort(

                _5923954032168212568.begin(), _5923954032168212568.end(), [

                ]
                        (

                                const pair<

                                        cv::

                                        Mat, double>

                                &_2654435866, const pair<
                                cv::
                                Mat, double>

                                &_2654435867) {

                    return _2654435866.second <

                           _2654435867.second;

                }
        );

        return

                _5923954032168212568[0]
                        .first;

    }

    bool System::_2016327979059285019( Frame &_175247759917) {
        bool _11093822302335;
        if (_175247759917.imageParams.isStereoCamera() || _175247759917.imageParams.isArray()) {
            _11093822302335 = _15186594243873234013(_175247759917);
        } else {
            _11093822302335 = _14954361871198802778(_175247759917);
        }

        if (!_11093822302335) return _11093822302335;

        curPose_ns = TheMap->keyframes.back().pose_f2g;
        cout << "cKfId:1434 " << currentKeyFrameId_105767 ;
        currentKeyFrameId_105767 = TheMap->keyframes.back().idx;
        cout << "cKfId:1436 " << currentKeyFrameId_105767 << "##" << "##" ;;

        _13028158409047949416 = true;

        return
                true;

    }

    bool

    System::

    _14954361871198802778(
            Frame &_175247759917) {

        if (
                !

                        sysMapIntializer->

                                process(

                                _175247759917, TheMap)

                )

            return

                    false;

        if

                (

                TheMap->

                        keyframes.size()
                >

                1 &&

                TheMap->map_points.size()

                >
                0) {

            _175247759917.ids =

                    TheMap->

                                    keyframes.back()

                            .ids;

        }

        globalOptimization();

        if

                (

                TheMap->

                        map_markers.size()

                ==

                0) {

            if

                    (TheMap->
                    map_points.size()
                     <

                     50) {

                TheMap->

                        clear();

                return

                        false;

            }

            float

                    _7847018097084079275 =

                    1. / TheMap->

                            getFrameMedianDepth(

                            TheMap->

                                            keyframes.front()

                                    .idx);

            cv::

            Mat _706246338944062 =

                    TheMap->

                                    keyframes.back()

                            .pose_f2g.inv();

            _706246338944062.col(

                            3)
                    .rowRange(
                            0, 3)

                    = _706246338944062.col(

                            3)
                              .rowRange(

                                      0, 3)

                      * _7847018097084079275;

            TheMap->
                            keyframes.back()

                    .pose_f2g =

                    _706246338944062.inv();

            for (

                auto &_175247759380: TheMap->

                    map_points) {

                _175247759380.scalePoint(
                        _7847018097084079275);

            }

        }

        curPose_ns =

                TheMap->

                                keyframes.back()

                        .pose_f2g;

        return

                true;

    }

    bool System::_15186594243873234013(Frame &inputFrame) {
        if (sysParams.KPNonMaximaSuppresion)
            inputFrame.nonMaximaSuppresion();
        int _8065948040949117953 = 0;
        for (size_t _2654435874 = 0; _2654435874 < inputFrame.und_kpts.size(); _2654435874++) {
            if (inputFrame.getDepth(_2654435874) > 0 &&
                inputFrame.imageParams.isClosePoint(inputFrame.getDepth(_2654435874)) &&
                !inputFrame.flags[_2654435874].is(Frame::FLAG_NONMAXIMA))
                _8065948040949117953++;
        }

        if (_8065948040949117953 < 100) return false;

        inputFrame.pose_f2g.setUnity();
        Frame &_16997199184281837438 = TheMap->addKeyFrame(inputFrame);

        for (size_t _2654435874 = 0; _2654435874 < inputFrame.und_kpts.size(); _2654435874++) {
            cv::Point3f _2654435881;

            if (inputFrame.getDepth(_2654435874) > 0 &&
                inputFrame.imageParams.isClosePoint(inputFrame.getDepth(_2654435874)) && !inputFrame.flags[_2654435874].is(
                        Frame::

                                            FLAG_NONMAXIMA)) {

                _2654435881 =
                        inputFrame.get3dStereoPoint(
                                _2654435874);

                auto
                        &_175247759380 =

                        TheMap->
                                addNewPoint(_16997199184281837438.fseq_idx);

                _175247759380.kfSinceAddition =

                        1;

                _175247759380.setCoordinates(

                        _2654435881);

                _175247759380.setStereo(
                        true);

                TheMap->

                        addMapPointObservation(
                        _175247759380.id, _16997199184281837438.idx, _2654435874);

                inputFrame.ids[

                        _2654435874]

                        =
                        _175247759380.id;
            }

        }

        for (const auto

                    &_markerObservation: inputFrame.markers) {

            TheMap->
                    addMarker(

                    _markerObservation);
        }

        return

                true;

    }

    string System::
    _2102381941757963317(

            uint64_t _11093822380353)
    const {

        string _706246330193866;

        string _46082576163156525 =
                "\x71\x77\x65\x72\x74\x79\x75\x69\x6f\x70\x61\x73\x64\x66\x67\x68\x6a\x6b\x6c\x7a\x78\x63\x76\x62\x6e\x6d\x31\x32\x33\x34\x35\x36\x37\x38\x39\x30\x51\x57\x45\x52\x54\x59\x55\x49\x4f\x50\x41\x53\x44\x46\x47\x48\x4a\x4b\x4c\x5a\x58\x43\x56\x42\x4e\x4d";

        uchar *_2654435884 =
                (

                        uchar *) &_11093822380353;

        int

                _2654435879 =
                sizeof(

                        _11093822380353)
                / sizeof(

                        uchar);

        for (

                int _2654435874 =

                        0;

                _2654435874 <

                _2654435879;

                _2654435874++) {

            _706246330193866.push_back(
                    _46082576163156525[

                            _2654435884[

                                    _2654435874]

                            % _46082576163156525.size()

                    ]

            );

        }

        return
                _706246330193866;

    }

    uint32_t System::getRefKeyFrameId(const Frame &frame_106140, const se3 &se3_107067) {
        int64_t refKeyFrameId = -1;

        if (sysParams.detectKeyPoints)
            refKeyFrameId = TheMap->getReferenceKeyFrame(frame_106140, 1);

        if (refKeyFrameId != -1){
            return refKeyFrameId;
        }

        if (TheMap->map_markers.size() == 0) {
            return currentKeyFrameId_105767;
        }

        vector<uint32_t> overlappedMarkers;

        for (auto _markerObservation: frame_106140.markers) {
            auto marker = TheMap->map_markers.find(_markerObservation.id);

            if (marker != TheMap->map_markers.end()) {
                if (marker->second.pose_g2m.isValid())
                    overlappedMarkers.push_back(_markerObservation.id);
            }
        }

        pair<uint32_t, float> result(std::numeric_limits<uint32_t>::max(), std::numeric_limits<float>::max());

        for (auto marker: overlappedMarkers)
            for (const auto &inputFrame: TheMap->map_markers[marker].frames) {
                auto _2654435869 = se3_107067.t_dist(TheMap->keyframes[inputFrame].pose_f2g);
                if (result.second > _2654435869) result = {inputFrame, _2654435869};
            }

        if(result.first != std::numeric_limits<uint32_t>::max()){
            return result.first;
        } else {
            return currentKeyFrameId_105767;
        }

    }

    std::vector<System::_4118122444908280734> System::_3473802998844434099(Frame &_16940374161494853219, se3 &_13011065492167565582, const std::
    set<uint32_t> &_16997249117545452056) {
        if(

                _16940374161494853219.ids.size()

                ==

                0)
            return {

            };

        if (
                TheMap->
                        TheKFDataBase.isEmpty()

                )

            return {
            };

        vector<

                uint32_t>
                _5288382201172378343 =

                TheMap->

                        relocalizationCandidates(
                        _16940374161494853219, _16997249117545452056);
        if

                (

                _5288382201172378343.size()

                ==

                0)

            return {

            };

        vector<

                System::

                _4118122444908280734> _1524129789187101628(

                _5288382201172378343.size()
        );

        FrameMatcher _16937386958649118140;

        _16937386958649118140.setParams(_16940374161494853219, FrameMatcher::

        MODE_ALL, sysParams.maxDescDistance * 2);

#pragma omp parallel for
        for (

                int _175247762874 =

                        0;

                _175247762874 <

                _5288382201172378343.size();

                _175247762874++

                ) {

            auto
                    _175247760268 = _5288382201172378343[
                    _175247762874];

            auto
                    &_3005399814901981436 =

                    TheMap->

                            keyframes[

                            _175247760268];

            _1524129789187101628[

                    _175247762874]

                    ._6116114700730085677 =

                    _16937386958649118140.match(
                            _3005399814901981436, FrameMatcher::

                            MODE_ASSIGNED);

            for (

                auto &_markerObservation: _1524129789187101628[_175247762874]

                    ._6116114700730085677) {

                std::
                swap(
                        _markerObservation.queryIdx, _markerObservation.trainIdx);

                _markerObservation.trainIdx =
                        _3005399814901981436.ids[

                                _markerObservation.trainIdx];

            }

            for (

                    int _2654435874 =

                            0;

                    _2654435874 < _1524129789187101628[_175247762874]
                            ._6116114700730085677.size();

                    _2654435874++

                    ) {

                auto

                        &_175247759380 =
                        _1524129789187101628[

                                _175247762874]

                                ._6116114700730085677[

                                _2654435874].trainIdx;

                if (

                        !

                                TheMap->

                                        map_points.is(

                                        _175247759380))

                    _1524129789187101628[

                            _175247762874]

                            ._6116114700730085677[

                            _2654435874]

                            .trainIdx =
                            -1;

                if (_175247759380 < TheMap->map_points.data_size() || TheMap->map_points[_175247759380].isBad())

                    _1524129789187101628[

                            _175247762874]

                            ._6116114700730085677[

                            _2654435874]
                            .trainIdx =
                            -1;

            }

            remove_unused_matches(

                    _1524129789187101628[
                            _175247762874]

                            ._6116114700730085677);

            if

                    (

                    _1524129789187101628[
                            _175247762874]

                            ._6116114700730085677.size()

                    <

                    25)
                continue;

            _1524129789187101628[

                    _175247762874]

                    ._3885067248075476027 =

                    _3005399814901981436.pose_f2g;

            PnPSolver::

            solvePnPRansac(
                    _16940374161494853219, TheMap, _1524129789187101628[

                            _175247762874]

                            ._6116114700730085677, _1524129789187101628[

                            _175247762874]

                            ._3885067248075476027);

            if
                    (

                    _1524129789187101628[
                            _175247762874]

                            ._6116114700730085677.size()
                    <

                    15)

                continue;

            _1524129789187101628[
                    _175247762874]

                    ._6116114700730085677 =

                    TheMap->
                            matchFrameToMapPoints(

                            TheMap->

                                    TheKpGraph.getNeighborsVLevel2(

                                    _175247760268, true), _16940374161494853219, _1524129789187101628[

                                    _175247762874]

                                    ._3885067248075476027, sysParams.maxDescDistance * 2, 2.5, true);

            if

                    (

                    _1524129789187101628[_175247762874]

                            ._6116114700730085677.size()

                    <

                    30)

                continue;

            PnPSolver::

            solvePnp(

                    _16940374161494853219, TheMap, _1524129789187101628[_175247762874]

                            ._6116114700730085677, _1524129789187101628[
                            _175247762874]
                            ._3885067248075476027);

            if

                    (

                    _1524129789187101628[

                            _175247762874]
                            ._6116114700730085677.size()
                    <

                    30)
                continue;

            _1524129789187101628[

                    _175247762874]

                    ._16902946305713852348 =

                    _16940374161494853219.ids;

            for (
                auto _46082575882272165: _1524129789187101628[

                    _175247762874]._6116114700730085677)

                _1524129789187101628[

                        _175247762874]
                        ._16902946305713852348[

                        _46082575882272165.queryIdx]

                        =

                        _46082575882272165.trainIdx;

        }

        std::

        remove_if(

                _1524129789187101628.begin(), _1524129789187101628.end(), [
                ]

                        (

                                const _4118122444908280734 &_2654435866) {

                    return _2654435866._6116114700730085677.size()

                           <=
                           30;

                }

        );

        std::

        sort(
                _1524129789187101628.begin(), _1524129789187101628.end(), [

                ]
                        (

                                const _4118122444908280734 &_2654435866, const _4118122444908280734 &_2654435867) {

                    return _2654435866._6116114700730085677.size()

                           >
                           _2654435867._6116114700730085677.size();

                }
        );

        return
                _1524129789187101628;

    }

    bool

    System::

    _3570943890084999391(

            Frame &_16940374161494853219, se3 &_13011065492167565582, const std::

    set<

            uint32_t>

    &_16997249117545452056) {

        auto
                _1524129789187101628 =

                _3473802998844434099(
                        _16940374161494853219, _13011065492167565582, _16997249117545452056);

        if (

                _1524129789187101628.size() ==

                0)

            return false;

        if (
                _1524129789187101628[

                        0]
                        ._6116114700730085677.size()

                >

                30) {

            _13011065492167565582 =

                    _1524129789187101628[

                            0]

                            ._3885067248075476027;

            _16940374161494853219.ids =
                    _1524129789187101628[

                            0]

                            ._16902946305713852348;

            return

                    true;

        } else

            return false;

    }

    bool

    System::

    _14569675007600066936(

            Frame &inputFrame, se3 &_13011065492167565582) {

        if

                (

                inputFrame.markers.size()

                ==

                0)

            return false;

        vector<
                uint32_t>

                _7895328205142007059;

        for (auto

                    &_markerObservation: inputFrame.markers) {

            auto

                    _8332348524113911167 =

                    TheMap->
                            map_markers.find(

                            _markerObservation.id);

            if (
                    _8332348524113911167 !=

                    TheMap->map_markers.end()
                    )

                if

                        (

                        _8332348524113911167->

                                second.pose_g2m.isValid()

                        )

                    _7895328205142007059.push_back(
                            _markerObservation.id);

        }

        if
                (
                _7895328205142007059.size()

                ==
                0)

            return false;

        _13011065492167565582 =
                TheMap->

                        getBestPoseFromValidMarkers(

                        inputFrame, _7895328205142007059, sysParams.aruco_minerrratio_valid);

        return

                _13011065492167565582.isValid();

    }

    bool
    System::

    _16487919888509808279(

            Frame &inputFrame, se3 &_13011065492167565582) {

        _13011065492167565582 =

                se3();

        if (

                sysParams.reLocalizationWithMarkers) {

            if

                    (

                    _14569675007600066936(

                            inputFrame, _13011065492167565582)

                    )
                return

                        true;

        }

        if (

                sysParams.reLocalizationWithKeyPoints) {

            if
                    (

                    _3570943890084999391(

                            inputFrame, _13011065492167565582))
                return

                        true;

        }

        return

                false;

    }
std::vector<cv::DMatch>System::_11946837405316294395(Frame &curFrame, Frame &_5918541169384278026, float _1686565542397313973, float _4500031049790251086) {
    std::vector<cv::DMatch> _6807036698572949990;
    for (size_t _2654435874 =0; _2654435874 < _5918541169384278026.ids.size(); _2654435874++) {
        uint32_t _11093822294347 = _5918541169384278026.ids[_2654435874];
        if (
            
                    _11093822294347 != std::

                    numeric_limits<

                            uint32_t>::max()) {

                if (

                        TheMap->

                                map_points.is(
                                _11093822294347)

                        ) {

                    MapPoint &_3005399799907669332 = TheMap->

                            map_points[

                            _11093822294347];

                    if

                            (

                            _3005399799907669332.isBad()
                            )

                        continue;

                    auto

                            _11093822300120 =
                            curFrame.project(

                                    _3005399799907669332.getCoordinates(), true, true);

                    if

                            (

                            isnan(

                                    _11093822300120.x))

                        continue;

                    float
                            _175247759755 =

                            curFrame.scaleFactors[_5918541169384278026.und_kpts[

                                    _2654435874]

                                    .octave];

                    int

                            _3005399801676750422 =

                            _5918541169384278026.und_kpts[

                                    _2654435874]
                                    .octave;

                    std::                                          // above

                    vector<

                            uint32_t>

                            _10924592426265627429 =
                            curFrame.getKeyPointsInRegion(
                                    _11093822300120, _4500031049790251086 * _175247759755, _3005399801676750422,
                                    _3005399801676750422);

                    float
                            _16940367568811467085 =

                            _1686565542397313973 + 0.01, _16992066385107317811 =

                            std::

                            numeric_limits<

                                    float>

                            ::

                            max();

                    uint32_t
                            _6806984971934960832 =

                            std::
                            numeric_limits<

                                    uint32_t>::

                            max();

                    for (
                        auto _175247760278: _10924592426265627429) {

                        if

                                (
                                curFrame.und_kpts[

                                        _175247760278]

                                        .octave ==

                                _5918541169384278026.und_kpts[_2654435874]

                                        .octave) {

                            float

                                    _16940392174182767813 = MapPoint::
                            getDescDistance(
                                    _5918541169384278026.desc, _2654435874, curFrame.desc, _175247760278);

                            if

                                    (

                                    _16940392174182767813 <

                                    _16940367568811467085) {

                                _16940367568811467085 =

                                        _16940392174182767813;

                                _6806984971934960832 =

                                        _175247760278;
                            } else if

                                    (

                                    _16940392174182767813 <
                                    _16992066385107317811) {

                                _16992066385107317811 =

                                        _16940392174182767813;

                            }

                        }

                    }

                    if
                            (

                            _6806984971934960832 !=

                            std::

                            numeric_limits<
                                    uint32_t>

                            ::

                            max()

                            &&
                            _16940367568811467085 <

                            0.7 * _16992066385107317811) {

                        cv::

                        DMatch _175247759376;

                        _175247759376.queryIdx =

                                _6806984971934960832;

                        _175247759376.trainIdx =

                                _3005399799907669332.id;

                        _175247759376.distance =

                                _16940367568811467085;

                        _6807036698572949990.push_back(

                                _175247759376);

                    }

                }

            }

        }

        filter_ambiguous_query(

                _6807036698572949990);

        return

                _6807036698572949990;

    }

    se3 System::_11166622111371682966(Frame &curFrame, se3 se3_143874) {
        std::vector<cv::DMatch> DMatchVector_637068;
        se3 curPoseF2g = se3_143874;

//       long value_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch()).count();
//       if(value_ms % 20 == 0){
//           cerr << "Evoke a test error" << endl;
//           curPoseF2g = se3();
//           throw std::invalid_argument("This is a test error");
//       }
//        cerr << "1: " << curPoseF2g << ", KFid = " << currentKeyFrameId_105767 << " / ";

        if(!curPoseF2g.isValid()){
            breakFlag = true;
            throw std::invalid_argument("se3 is empty, reinitialize");
        }
        if (TheMap->map_points.size() > 0) {
            cv::Mat convertedcurPoseF2g = curPoseF2g.convert();

            // EDIT: Perform a check here to ensure it won't crash
            if (!_14463320619150402643.empty() && !convertedcurPoseF2g.empty()) {
                curPoseF2g = _14463320619150402643 * convertedcurPoseF2g;
            } else {
                throw std::invalid_argument("se3 is empty!");
            }

            curFrame.pose_f2g = curPoseF2g;
            DMatchVector_637068 = _11946837405316294395(curFrame, lastFrame,
                                                        sysParams.maxDescDistance * 1.5,
                                                        sysParams.projDistThr);
            int PnPResult = 0;

            if (DMatchVector_637068.size() > 30) {
                auto estimatedPose = curPoseF2g;
                PnPResult = PnPSolver::solvePnp(curFrame, TheMap, DMatchVector_637068,
                                                           estimatedPose, currentKeyFrameId_105767);

                if (PnPResult > 30){
                    curPoseF2g = estimatedPose;
                }

            } else {
                FrameMatcher _16997326787393468537(FrameMatcher::TYPE_FLANN);
                _16997326787393468537.setParams(TheMap->keyframes[currentKeyFrameId_105767],
                                                FrameMatcher::MODE_ASSIGNED, sysParams.maxDescDistance * 2,
                                                0.6, true, 3);
                DMatchVector_637068 = _16997326787393468537.match(curFrame, FrameMatcher::MODE_ALL);

                if (DMatchVector_637068.size() > 30) {
                    for (auto &_markerObservation: DMatchVector_637068)
                        _markerObservation.trainIdx = TheMap->keyframes[currentKeyFrameId_105767].ids[_markerObservation.trainIdx];

                    auto estimatedPose = curPoseF2g;
                    PnPResult = PnPSolver::solvePnp(curFrame, TheMap, DMatchVector_637068,
                                                               estimatedPose, currentKeyFrameId_105767);
                    if (PnPResult > 30){
                        curPoseF2g = estimatedPose;;
                    }
                } else PnPResult = 0;
            }

            float _3763415994652820314;
            if (PnPResult > 30) {
                _3763415994652820314 = 4;
                for (auto _markerObservation: DMatchVector_637068) {
                    TheMap->map_points[_markerObservation.trainIdx].lastFIdxSeen = curFrame.fseq_idx;
                    TheMap->map_points[_markerObservation.trainIdx].setVisible();
                }
            } else {
                DMatchVector_637068.clear();
                _3763415994652820314 = sysParams.projDistThr;
            }

            auto _3521005873836563963 = TheMap->matchFrameToMapPoints(
                    TheMap->TheKpGraph.getNeighborsVLevel2(currentKeyFrameId_105767, true),
                    curFrame,
                    curPoseF2g,
                    sysParams.maxDescDistance * 2,
                    _3763415994652820314, true);

            DMatchVector_637068.insert(DMatchVector_637068.end(), _3521005873836563963.begin(),
                                       _3521005873836563963.end());

            filter_ambiguous_query(DMatchVector_637068);
        }

        int matchNumber = PnPSolver::solvePnp(curFrame, TheMap, DMatchVector_637068, curPoseF2g,
                                              currentKeyFrameId_105767);
        bool foundPosition = false;
        if (curFrame.markers.size() > 0) {
            int validMarkerCount = 0;

            for (size_t _2654435874 = 0; _2654435874 < curFrame.markers.size(); _2654435874++) {
                auto marker_1109 = TheMap->map_markers.find(curFrame.markers[_2654435874].id);

                if (marker_1109 == TheMap->map_markers.end())
                    continue;

                if (!marker_1109->second.pose_g2m.isValid())
                    continue;

                validMarkerCount++;

                if (curFrame.markers[_2654435874].poses.err_ratio < sysParams.aruco_minerrratio_valid)
                    continue;

                foundPosition = true;

                break;
            }

            if (validMarkerCount > 1)
                foundPosition = true;

        }

        if (matchNumber < 30 && !foundPosition) {
            return se3();
        }

        for (size_t _2654435874 = 0; _2654435874 < DMatchVector_637068.size(); _2654435874++) {
            TheMap->map_points[DMatchVector_637068[_2654435874].trainIdx].setSeen();

            curFrame.ids[DMatchVector_637068[_2654435874].queryIdx] = DMatchVector_637068[_2654435874].trainIdx;

            if (DMatchVector_637068[_2654435874].imgIdx == -1)
                curFrame.flags[DMatchVector_637068[_2654435874].queryIdx].set(Frame::FLAG_OUTLIER, true); // set
        }
        return curPoseF2g;
    }

    void System::_14031550457846423181(cv::Mat &_46082544231248938, float _9971115036363993554) const {
        int _706246330297760 = float(_46082544231248938.cols) / 640.f;
        cv::Point2f _46082575822903876(_706246330297760, _706246330297760);
        bool _16987668682974831349 = false;

        if(trackingState == STATE_TRACKING) {
            for (size_t _2654435874 = 0; _2654435874 < curFrame.ids.size(); _2654435874++)
                if(curFrame.ids[_2654435874]!=std::numeric_limits<uint32_t>::max()) {
                    if(!TheMap->map_points.is(curFrame.ids[_2654435874]))
                        continue;

                    cv::
                    Scalar _46082574599890393(

                            0, 255, 0);

                    if

                            (

                            !
                                    TheMap->

                                            map_points[curFrame.ids[

                                            _2654435874]

                                    ]

                                            .isStable()

                            ) {

                        _46082574599890393 =
                                cv::

                                Scalar(
                                        0, 0, 255);

                    }

                    cv::

                    rectangle(

                            _46082544231248938, _9971115036363993554 * (
                                    curFrame.kpts[_2654435874]

                                    - _46082575822903876), _9971115036363993554 * (

                                    curFrame.kpts[
                                            _2654435874]

                                    + _46082575822903876), _46082574599890393, _706246330297760);

                } else if (

                        _16987668682974831349) {

                    cv::
                    Scalar _46082574599890393(

                            255, 0, 0);

                    cv::

                    rectangle(

                            _46082544231248938, _9971115036363993554 * (

                                    curFrame.kpts[

                                            _2654435874]

                                    - _46082575822903876), _9971115036363993554 * (curFrame.kpts[
                                                                                           _2654435874]

                                                                                   + _46082575822903876),
                            _46082574599890393, _706246330297760);
                }

        } else if (

                TheMap->

                        isEmpty()

                ) {

            for (

                auto

                        _2654435881: curFrame.kpts)

                cv::

                rectangle(_46082544231248938, _9971115036363993554 * (

                        _2654435881 - _46082575822903876), _9971115036363993554 * (

                        _2654435881 + _46082575822903876), cv::
                          Scalar(

                        255, 0, 0), _706246330297760);

        }

        for (

            auto _5221496220235804833: curFrame.markers) {

            cv::

            Scalar _46082574599890393 =

                    cv::

                    Scalar(

                            0, 244, 0);

            if (
                    TheMap->

                            map_markers.count(_5221496220235804833.id)
                    !=

                    0) {

                if (

                        TheMap->

                                        map_markers.at(

                                        _5221496220235804833.id)
                                .pose_g2m.isValid()

                        )

                    _46082574599890393 =

                            cv::

                            Scalar(

                                    255, 0, 0);

                else
                    _46082574599890393 =
                            cv::

                            Scalar(

                                    0, 0, 255);

            }

            for (

                auto &_2654435881: _5221496220235804833.corners)
                _2654435881 *=

                        _9971115036363993554;

            for (auto &_2654435881: _5221496220235804833.und_corners)

                _2654435881 *=

                        _9971115036363993554;

            _5221496220235804833.draw(
                    _46082544231248938, _46082574599890393);

        }

    }

    void

    System::

    globalOptimization() {

        auto

                _11093822300040 =

                GlobalOptimizer::
                create(

                        sysParams.global_optimizer);

        GlobalOptimizer::

        ParamSet _3005399798454910266(

                debug::

                Debug::

                getLevel()
                >=

                11);

        _3005399798454910266.fixFirstFrame =

                true;

        _3005399798454910266.nIters =

                20;

        _3005399798454910266.markersOptWeight =

                getParams()
                        .markersOptWeight;

        _3005399798454910266.minMarkersForMaxWeight =

                getParams()

                        .minMarkersForMaxWeight;

        _3005399798454910266.InPlaneMarkers =

                getParams()

                        .inPlaneMarkers;
        _11093822300040->

                optimize(
                TheMap, _3005399798454910266);

        TheMap->

                removeBadAssociations(

                _11093822300040->

                        getBadAssociations(), 2);

    }

    uint32_t

    System::

    getLastProcessedFrame()

    const {

        return curFrame.fseq_idx;

    }

    void System::

    setMode(

            MODES _706246332824366) {

        sysMode =

                _706246332824366;

    }

    void System::clear() {
        sysMapManager = std::make_shared<MapManager>();
        _13028158409047949416 = false;

        trackingState = STATE_LOST;
        TheMap.reset();

        sysMapIntializer = std::make_shared<MapInitializer>();
        _14463320619150402643 = cv::Mat();
        _10558050725520398793 = -1;
    }

    void System::saveToFile(string _16997227483604144380) {
        waitForFinished();
        fstream _706246330143775(_16997227483604144380, ios_base::binary | ios_base::out);

        if (!_706246330143775)
            throw std::runtime_error(string(__PRETTY_FUNCTION__)

                                     +
                                     "\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x6f\x70\x65\x6e\x20\x66\x69\x6c\x65\x20\x66\x6f\x72\x20\x77\x72\x69\x74\x69\x6e\x67\x3a" +
                                     _16997227483604144380);

        io_write<uint64_t>(182313, _706246330143775);

        TheMap->toStream(_706246330143775);

        sysParams.toStream(_706246330143775);
        _706246330143775.write((char *) &curPose_ns, sizeof(curPose_ns));

        _706246330143775.write((char *) &currentKeyFrameId_105767, sizeof(currentKeyFrameId_105767));

        _706246330143775.write(
                (

                        char *)

                        &_13028158409047949416, sizeof(

                        _13028158409047949416)

        );

        _706246330143775.write(
                (

                        char *)

                        &trackingState, sizeof(

                        trackingState)
        );

        _706246330143775.write(

                (

                        char *)

                        &sysMode, sizeof(

                        sysMode)

        );

        curFrame.toStream(

                _706246330143775);

        lastFrame.toStream(

                _706246330143775);
        sysFrameExtractor->

                toStream(

                _706246330143775);

        sysMapManager->

                toStream(
                _706246330143775);

        _1320287184975591154->

                toStream(
                _706246330143775);

        toStream__(

                _14463320619150402643, _706246330143775);

        _706246330143775.write(

                (
                        char *)

                        &_10558050725520398793, sizeof(
                        _10558050725520398793)

        );

        _706246330143775.write(

                (

                        char *)

                        &_13033649816026327368, sizeof(

                        _13033649816026327368)

        );

        _706246330143775.flush();

    }

    void System::readFromFile(string _16997227483604144380) {

        ifstream _706246330143775(_16997227483604144380, ios::binary);
        if (!_706246330143775)
            throw std::runtime_error(
                    string(__PRETTY_FUNCTION__) + R"(could not open file for reading:)" + _16997227483604144380);
        if (io_read<uint64_t>(_706246330143775) != 182313)
            throw std::runtime_error(string(__PRETTY_FUNCTION__) + R"(invalid file type:)" + _16997227483604144380);

        TheMap = std::make_shared<Map>();
        TheMap->fromStream(_706246330143775);

        sysParams.fromStream(_706246330143775);
        _706246330143775.read((char *) &curPose_ns, sizeof(curPose_ns));
        _706246330143775.read((char *) &currentKeyFrameId_105767, sizeof(currentKeyFrameId_105767));
        _706246330143775.read((char *) &_13028158409047949416, sizeof(_13028158409047949416));
        _706246330143775.read((char *) &trackingState, sizeof(trackingState));
        _706246330143775.read((char *) &sysMode, sizeof(sysMode));
        curFrame.fromStream(_706246330143775);
        lastFrame.fromStream(_706246330143775);
        sysFrameExtractor->fromStream(_706246330143775);

        sysMapManager->fromStream(_706246330143775);
        _1320287184975591154->fromStream(_706246330143775);
        fromStream__(_14463320619150402643, _706246330143775);

        _706246330143775.read((char *) &_10558050725520398793, sizeof(_10558050725520398793));

        _706246330143775.read((char *) &_13033649816026327368, sizeof(_13033649816026327368));
    }

}