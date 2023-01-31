#include "utils/mapmanager.h"
#include "basictypes/osadapter.h"
#include "map.h"
#include "utils/system.h"
#include "optimization/ippe.h"
#include "basictypes/misc.h"
#include "basictypes/io_utils.h"
#include "basictypes/timers.h"
#include "optimization/globaloptimizer.h"
#include "optimization/pnpsolver.h"
#include "basictypes/minmaxbags.h"
#include <xflann/xflann.h>
#include "basictypes/hash.h"
#include "utils/framematcher.h"

#ifdef USE_OMP
#include <omp.h>
#endif

namespace tslam {

    MapManager::MapManager() {
        _curState = IDLE;
        _TheLoopDetector_141391 = createLoopDetector_464485(_loopClosureEnabled_100469);
    }

    MapManager::~MapManager() {
        stop();
    }

    void MapManager::setParams(std::shared_ptr<Map> map_110938, bool EnableLoopClosure_906574) {
        TheMap = map_110938;
        _loopClosureEnabled_100469 = EnableLoopClosure_906574;
        _TheLoopDetector_141391 = createLoopDetector_464485(_loopClosureEnabled_100469);
        _TheLoopDetector_141391->setParams(map_110938);
    }

    uint32_t MapManager::getLastAddedKeyFrameIdx() const {
        return _lastAddedKeyFrame;
    }

    bool MapManager::hasMap() const {
        return !(!TheMap);
    }

    std::

    shared_ptr<

            BaseLoopDetector>

    MapManager::
    createLoopDetector_464485(

            bool _18278402211387234209) {

        if (
                _18278402211387234209)
            return

                    std::

                    make_shared<

                            LoopDetector>

                            ();

        else
            return
                    std::

                    make_shared<
                            UselessLoopDetector>

                            ();

    }

    int MapManager::newFrame(

            Frame &_175247760268, int32_t currentKeyFrameId) {

        _CurkeyFrame =

                currentKeyFrameId;

        _9728777609121731073++;

        bigChangeHasHappen =

                false;

        int

                _3209905912317706228 =

                0;

        if

                (

                _curState.load()

                ==

                IDLE) {

#pragma message "warning : in non-sequential mode detected markers in loop closure are not proceesed properly?"
            if (System::getParams().reLocalizationWithMarkers && !System::getParams().isInstancing) {
                _LoopClosureInfo = _TheLoopDetector_141391->detectLoopFromMarkers(_175247760268, currentKeyFrameId);
            }


            if

                    (
                    _LoopClosureInfo.foundLoop()
                    ) {
                _TheLoopDetector_141391->
                        correctMap(
                        _LoopClosureInfo);

                loopClosurePostProcessing(
                        _175247760268, _LoopClosureInfo);

                bigChangeHasHappen =

                        true;

                _3209905912317706228 =
                        2;

            } else {

                if

                        (

                        _668185896188051300(
                                _175247760268, currentKeyFrameId)

                        ) {

                    _9728777609121731073 = 0;

                    Frame *_46082575805180420 =
                            new Frame(
                                    _175247760268);

                    keyframesToAdd.push(

                            _46082575805180420);

                    if

                            (

                            !_4098354751575524583.joinable()
                            )
                        mainFunction();

                    _3209905912317706228 =

                            1;

                } else {

                }

            }

        } else {

            if

                    (

                    _668185896188051300(

                            _175247760268, currentKeyFrameId)

                    ) {

            }

            if (System::getParams().reLocalizationWithMarkers && !System::getParams().isInstancing) {
                _LoopClosureInfo = _TheLoopDetector_141391->detectLoopFromMarkers(_175247760268, currentKeyFrameId);
            }

            if (

                    _LoopClosureInfo.foundLoop()) {

            }

        }

        return

                _3209905912317706228;

    }

    bool MapManager::mapUpdate() {
        if (_curState != WAITINGFORUPDATE) return false;

        _curState = WORKING;
        TheMap->lock(__FUNCTION__, __FILE__, __LINE__);
        if (_LoopClosureInfo.foundLoop()) {
            _TheLoopDetector_141391->correctMap(_LoopClosureInfo);
            loopClosurePostProcessing(TheMap->keyframes[_lastAddedKeyFrame], _LoopClosureInfo);
            bigChangeHasHappen = true;
        } else {
            vector<std::pair<uint32_t, uint32_t> > BadAssociations;
            if (Gopt) { //if we just wake up, from a loadFromStream, this object is not created
                Gopt->getResults(TheMap);
                BadAssociations = Gopt->getBadAssociations();
                Gopt = nullptr;
            }
            TheMap->removeBadAssociations(BadAssociations, System::getParams().minNumProjPoints);
        }
        for (auto p: PointsToRemove)
            if (TheMap->map_points.is(p))
                TheMap->removePoint(p);

        PointsToRemove.clear();

        TheMap->removeKeyFrames(KeyFramesToRemove, System::getParams().minNumProjPoints);

        if (_hasMapBeenScaled) {
            bigChangeHasHappen = true;
        }

        _lastAddedKFPose = TheMap->keyframes[_lastAddedKeyFrame].pose_f2g;

        TheMap->removeWeakConnections(_CurkeyFrame, 8);

        TheMap->unlock(__FUNCTION__, __FILE__,__LINE__);

        PointsToRemove.clear();
        KeyFramesToRemove.clear();

        _curState = IDLE;
        return true;
    }

    void

    MapManager::
    start() {
        if

                (_4098354751575524583.joinable()

                )

            return;

        _4090819199315697352 = false;

        _4098354751575524583 =
                std::

                thread(

                        [

                                this] {

                            this->

                                    _8669746328630631075();

                        }

                );
    }

    void
    MapManager::

    stop() {

        if

                (

                _4098354751575524583.joinable()
                ) {

            _4090819199315697352 =
                    true;

            _hurryUp =

                    false;

            keyframesToAdd.push(

                    NULL);

            _4098354751575524583.join();

        }

    }

    void

    MapManager::

    reset() {

        if

                (

                _4098354751575524583.joinable()

                ) {

            _4090819199315697352 =

                    true;

            keyframesToAdd.push(

                    NULL);

            _4098354751575524583.join();

        }
        _4090819199315697352 =

                false;

        keyframesToAdd.clear();

        _curState =

                IDLE;

        TheMap.reset();

        _CurkeyFrame =

                std::
                numeric_limits<

                        uint32_t>

                ::

                max();

        Gopt.reset();

        PointsToRemove.clear();

        KeyFramesToRemove.clear();

        std::

        map<

                uint32_t, uint32_t

        >
                _14515052224023340288;

        _TheLoopDetector_141391 =

                createLoopDetector_464485(

                        _loopClosureEnabled_100469);

        _LoopClosureInfo =

                LoopDetector::

                LoopClosureInfo();

        _lastAddedKeyFrame =

                std::

                numeric_limits<

                        uint32_t>

                ::

                max();

        _hasMapBeenScaled =

                false;

        _hurryUp =

                false;

    }

    Frame &MapManager::addKeyFrame(Frame *newPtrFrame) {
        auto _17591916323427771156 = [this]() {
            int _8650310500306039378 = 0;

            for (auto &m: TheMap->map_markers)
                if (m.second.pose_g2m.isValid()) _8650310500306039378++;
            return _8650310500306039378;
        };

        Frame &keyframe_169372 = TheMap->addKeyFrame(*newPtrFrame);
        newInsertedKeyFrames.push(keyframe_169372.idx);
        _lastAddedKeyFrame = keyframe_169372.idx;
        youngKeyFrames.insert({keyframe_169372.idx, 0});

        vector<uint32_t> framesToRemove_169972;
        for (auto &kf: youngKeyFrames) {
            kf.second++;
            if (kf.second > 3) framesToRemove_169972.push_back(kf.first);
        }

        for (auto r: framesToRemove_169972) youngKeyFrames.erase(r);

        if (System::getParams().KPNonMaximaSuppresion) keyframe_169372.nonMaximaSuppresion();

        int _1515507901219546526 = 0;

        for (size_t _2654435874 = 0; _2654435874 < keyframe_169372.ids.size(); _2654435874++) {
            if (keyframe_169372.ids[_2654435874] != std::numeric_limits<uint32_t>::max()) {
                TheMap->addMapPointObservation(keyframe_169372.ids[_2654435874], keyframe_169372.idx, _2654435874);
                _1515507901219546526++;
            }
        }

        _hasMapBeenScaled = false;

        bool _14173211929012135714 = false;

        if (_17591916323427771156() == 0 && TheMap->map_points.size() != 0)
            _14173211929012135714 = true;

        for (size_t _markerObservation = 0; _markerObservation < keyframe_169372.markers.size(); _markerObservation++) {
            auto &markerAdded = TheMap->addMarker(keyframe_169372.markers[_markerObservation]);
            TheMap->addMarkerObservation(markerAdded.id, keyframe_169372.idx);

            if (!markerAdded.pose_g2m.isValid()) {
                if (!_14173211929012135714 && System::getParams().aruco_allowOneFrameInitialization) {
                    if (keyframe_169372.markers[_markerObservation].poses.err_ratio >
                        System::getParams().aruco_minerrratio_valid)
                        markerAdded.pose_g2m =
                                keyframe_169372.pose_f2g.inv() * keyframe_169372.markers[_markerObservation].poses.sols[0];

                }
            }
            auto _5829010262908049596 = [](Se3Transform &_2654435866, Se3Transform &_2654435867) {
                auto _175247759816 = _2654435866(cv::Range(0, 3), cv::Range(3, 4));
                auto

                        _175247759819 = _2654435867(cv::Range(0, 3), cv::Range(3, 4));

                return cv::norm(_175247759816 - _175247759819);
            };

            if (!markerAdded.pose_g2m.isValid() &&
                markerAdded.frames.size() >= size_t(System::getParams().aruco_minNumFramesRequired)) {
                vector<uint32_t> _6807035637074954094(markerAdded.frames.begin(),
                                                      markerAdded.frames.end());
                vector<bool> _4942080627572011540(_6807035637074954094.size(), false);

                vector<uint32_t> _347298374087418072;
                _347298374087418072.reserve(markerAdded.frames.size());

                for (

                        size_t i =

                                0;

                        i <
                        _6807035637074954094.size();

                        i++

                        ) {

                    if

                            (

                            !

                                    _4942080627572011540[

                                            i]

                            ) {

                        pair<

                                int, float>

                                best(

                                -1, std::

                                numeric_limits<

                                        float>

                                ::

                                lowest()

                        );

                        for (

                                size_t j =

                                        i + 1;

                                j <

                                _6807035637074954094.size(); j++

                                ) {

                            if (

                                    !

                                            _4942080627572011540[

                                                    j]

                                    ) {

                                float
                                        d =

                                        _5829010262908049596(TheMap->

                                                keyframes[

                                                                     _6807035637074954094[

                                                                             i]

                                                             ]
                                                                     .pose_f2g, TheMap->

                                                keyframes[

                                                                     _6807035637074954094[
                                                                             j]
                                                             ]

                                                                     .pose_f2g);

                                if

                                        (d > System::

                                getParams()

                                        .minBaseLine &&
                                         d >

                                         best.first)

                                    best =

                                            {

                                                    j, d};

                            }

                        }

                        if
                                (
                                best.first !=

                                -1) {

                            _347298374087418072.push_back(

                                    _6807035637074954094[
                                            i]

                            );
                            _347298374087418072.push_back(

                                    _6807035637074954094[
                                            best.first]

                            );

                            _4942080627572011540[

                                    i]
                                    =

                                    true;

                            _4942080627572011540[

                                    best.first] =

                                    true;

                        }

                    }

                }

                if

                        (

                        _347298374087418072.size()

                        >=
                        size_t(

                                System::

                                getParams()

                                        .aruco_minNumFramesRequired)
                        ) {

                    vector<tslam::

                    MarkerObservation>

                            _16750267944260729636;

                    vector<
                            se3>

                            _11822840474894279984;

                    for (
                        auto f: _347298374087418072) {

                        _16750267944260729636.push_back(TheMap->

                                keyframes[

                                                                f]

                                                                .getMarker(

                                                                        markerAdded.id)

                        );
                        _11822840474894279984.push_back(

                                TheMap->

                                        keyframes[

                                        f]

                                        .pose_f2g);

                    }

                    auto

                            _706246335742885 =

                            ARUCO_bestMarkerPose(

                                    _16750267944260729636, _11822840474894279984,
                                    keyframe_169372.imageParams.undistorted()

                            );

                    if

                            (

                            !_706246335742885.empty()
                            ) {

                        markerAdded.pose_g2m = _706246335742885;

                    }

                }

            }

        }

        if (

                _14173211929012135714 &&
                _17591916323427771156()

                >

                0) {

            pair<
                    double, double>

                    _6868692417182700890(

                    0, 0);

            for (auto &m: keyframe_169372.markers) {
                auto
                        &mapMarker = TheMap->
                        map_markers.at(

                        m.id);

                if

                        (
                        !mapMarker.pose_g2m.isValid()

                        )

                    continue;

                cv::

                Point2f center(

                        0, 0);

                for (

                    auto p: m.und_corners)

                    center + p;

                center *=

                        1. / 4.;

                double

                        maxDist =
                        std::

                        numeric_limits<

                                double>

                        ::
                        min();

                for (

                    auto p: m.und_corners)

                    maxDist =

                            std::
                            max(

                                    cv::

                                    norm(

                                            center - p), maxDist);

                vector<

                        uint32_t>

                        p3dis = keyframe_169372.getIdOfPointsInRegion(
                        center, maxDist);

                if

                        (

                        p3dis.size()

                        <

                        5)

                    continue;

                double

                        distSum = 0;

                for (
                    auto pid: p3dis) {

                    distSum +=

                            cv::

                            norm(

                                    keyframe_169372.pose_f2g * TheMap->

                                            map_points[

                                            pid]
                                            .getCoordinates()

                            );

                }

                double

                        avrgPointDist =

                        distSum / double(p3dis.size()

                        );

                cv::

                Mat f2m =

                        keyframe_169372.pose_f2g * mapMarker.pose_g2m;

                double

                        frameDist =

                        cv::

                        norm(

                                f2m.rowRange(

                                                0, 3)
                                        .colRange(

                                                3, 4)
                        );
                _6868692417182700890.first +=

                        frameDist / avrgPointDist;

                _6868692417182700890.second++;

            }

            if
                    (

                    _6868692417182700890.second ==

                    0) {

                for (

                    auto &m: TheMap->
                        map_markers)

                    m.second.pose_g2m =
                            se3();

            } else {

                double

                        _17370277987955713200 =

                        _6868692417182700890.first / _6868692417182700890.second;

                TheMap->

                        scale(

                        _17370277987955713200);

                _10758134674558762512(

                        10);

                _hasMapBeenScaled =

                        true;

            }

        }

        return
                keyframe_169372;

    }

    void MapManager::mainFunction() {
        _hurryUp = false;

        //first check if any new frame to be inserted
        Frame *newPtrFrame;
        keyframesToAdd.pop(newPtrFrame);
        if (newPtrFrame == NULL) return;

        _curState = WORKING;

        TheMap->lock(__FUNCTION__, __FILE__, __LINE__);
        Frame &keyframe_169372 = addKeyFrame(newPtrFrame);

        delete newPtrFrame;

        if (System::getParams().reLocalizationWithKeyPoints && !System::getParams().isInstancing) {
            _LoopClosureInfo = _TheLoopDetector_141391->detectLoopFromKeyPoints(keyframe_169372,_CurkeyFrame);
        }


        TheMap->unlock(__FUNCTION__, __FILE__, __LINE__);

        PointsToRemove = _8352839093262355382();

        TheMap->removePoints(PointsToRemove.begin(), PointsToRemove.end(), false);

        TheMap->lock(__FUNCTION__, __FILE__, __LINE__);

        int nn = 20;

        if (keyframe_169372.imageParams.isStereoCamera()) {
            nn = 5;

            for (const auto &nmp: _8820655757626307961(keyframe_169372)) {
                auto &mPoint = TheMap->addNewPoint(keyframe_169372.fseq_idx);
                mPoint.setStereo(true);

                mPoint.setCoordinates(nmp.pose);

                for (auto obs: nmp.frame_kpt) TheMap->addMapPointObservation(mPoint.id, obs.first, obs.second);
            }
        }

        auto newPoints = createNewPoints(keyframe_169372, nn, System::getParams().maxNewPoints);
        for (const auto &nmp: newPoints) {
            auto &mPoint = TheMap->addNewPoint(keyframe_169372.fseq_idx);

            mPoint.setCoordinates(nmp.pose);

            for (auto obs: nmp.frame_kpt) TheMap->addMapPointObservation(mPoint.id, obs.first, obs.second);
        }

        TheMap->unlock(__FUNCTION__, __FILE__, __LINE__);

        if (keyframesToAdd.empty()) {

            TheMap->lock(__FUNCTION__, __FILE__, __LINE__);

            auto _16937196451844060927 = _17400054198872595804(keyframe_169372);

            PointsToRemove.insert(
                    PointsToRemove.end(),
                    _16937196451844060927.begin(),
                    _16937196451844060927.end()
            );

            TheMap->unlock(__FUNCTION__, __FILE__, __LINE__);
        }

        {
            int _706246332319248 = 0;

            for (const auto &mp: TheMap->map_points)
                if (mp.isBad()) _706246332319248++;
        }

        if (!_hurryUp && TheMap->keyframes.size() > 1) {
            _11362629803814604768(keyframe_169372.idx);
        }

        if (!_hurryUp) {
            KeyFramesToRemove = keyFrameCulling(keyframe_169372.idx);
            for (auto kf: KeyFramesToRemove)
                TheMap->keyframes[kf].setBad(true);
        }
        _curState = WAITINGFORUPDATE;
    }

    Se3Transform MapManager::getLastAddedKFPose() {
        return _lastAddedKFPose;
    }

    bool MapManager::bigChange() const {
        return bigChangeHasHappen;
    }

    void

    MapManager::
    _8669746328630631075() {

        while (!

                _4090819199315697352) {

            mainFunction();

        }

    }

    set<uint32_t> MapManager::keyFrameCulling(uint32_t keyframe_idx) {
        set<uint32_t> KFtoRemove;

        // keep deleting the new frame during instancing
        if (System::getParams().isInstancing) {
            // if there're > the specified number of new added frame, remove them
            while (newInsertedKeyFrames.size() > 20) {
                auto kfIdxToBeRemoved = newInsertedKeyFrames.front();
                newInsertedKeyFrames.pop();
                KFtoRemove.insert(kfIdxToBeRemoved);
            }

        } else if (System::getParams().detectMarkers && TheMap->map_markers.size() != 0) {
            vector<uint32_t> NotRedundant;

            KFtoRemove = _5122744303662631154(keyframe_idx);

            for (auto kf: KFtoRemove) {
                const auto &ThisKFrame = TheMap->keyframes[kf];
                std::set<uint32_t> allFrames;

                for (const auto &m: ThisKFrame.markers)
                    for (auto f: TheMap->map_markers[m.id].frames)
                        allFrames.insert(f);

                allFrames.erase(kf);

                bool isRedundant = false;

                for (auto fidx: allFrames) {
                    int nMarkersCommon = 0;
                    for (const auto &m: TheMap->keyframes[fidx].markers)
                        if (ThisKFrame.getMarkerIndex(m.id) != -1)
                            nMarkersCommon++;
                    if (nMarkersCommon == ThisKFrame.markers.size()) {
                        isRedundant = true;
                        break;
                    }
                }

                if (!isRedundant)
                    NotRedundant.push_back(kf);
            }

            for (auto f: NotRedundant)
                KFtoRemove.erase(f);
        } else if (System::getParams().detectKeyPoints) KFtoRemove = _5122744303662631154(keyframe_idx);

        else if (System::getParams().detectMarkers) KFtoRemove = _17920146964341780569(keyframe_idx);

        return KFtoRemove;

    }

    set<
            uint32_t>

    MapManager::
    _17920146964341780569(

            uint32_t keyframe_idx) {

        auto
                _706246308970949 =

                [

                ]

                        (

                                uint32_t _2654435866, uint32_t

                        _2654435867) {

                    if (
                            _2654435866 >

                            _2654435867)

                        swap(
                                _2654435866, _2654435867);

                    uint64_t _11093821964632;

                    uint32_t

                            *_6807034398601546557 =

                            (
                                    uint32_t *)

                                    &_11093821964632;
                    _6807034398601546557[
                            0]

                            =

                            _2654435867;

                    _6807034398601546557[

                            1]
                            =

                            _2654435866;

                    return
                            _11093821964632;

                };

        auto

                _46082575804458778 =

                TheMap->
                        TheKpGraph.getNeighbors(

                        keyframe_idx);

        _46082575804458778.erase(

                TheMap->

                                keyframes.front()

                        .idx);

        std::

        map<

                uint64_t, float>
                _124580014028079534;

        vector<
                uint32_t> _3005399810248445333(
                _46082575804458778.begin(), _46082575804458778.end()
        );

        for (

                size_t i =

                        0;

                i <
                _3005399810248445333.size();

                i++

                ) {

            const auto &fi =

                    TheMap->

                            keyframes[

                            _3005399810248445333[
                                    i]

                    ];
            for (

                    size_t j =

                            i + 1;

                    j <

                    _3005399810248445333.size();
                    j++
                    ) {

                const auto &fj =

                        TheMap->

                                keyframes[

                                _3005399810248445333[

                                        j]
                        ];

                _124580014028079534[

                        _706246308970949(

                                _3005399810248445333[

                                        i], _3005399810248445333[

                                        j]

                        )

                ]

                        =

                        cv::

                        norm(

                                fi.pose_f2g.getTvec()

                                - fj.pose_f2g.getTvec()

                        );

            }
        }

        std::

        map<
                uint32_t, set<
                        uint32_t>

        >

                _13773082371983786779;

        for (

            auto fidx: _46082575804458778) {

            for (

                auto m: TheMap->

                    keyframes[
                    fidx]

                    .markers)

                _13773082371983786779[

                        m.id]
                        .insert(

                                fidx);

        }

        auto

                _10086624862567280113 =

                [

                        &]
                        (

                                uint32_t _706246330143240, const set<

                                uint32_t>

                        &_3005401603918369918) {

                    float

                            _706246353090457 =

                            0;
                    for (

                        auto f2idx: _3005401603918369918) {

                        if (

                                f2idx !=

                                _706246330143240)
                            _706246353090457 +=

                                    _124580014028079534[

                                            _706246308970949(
                                                    _706246330143240, f2idx)];

                    }

                    return

                            _706246353090457;
                };

        std::

        map<

                uint32_t, set<
                        uint32_t>

        >
                _12358233879185425501;

        for (
            auto mf: _13773082371983786779) {
            if
                    (
                    mf.second.size()

                    <=

                    size_t(
                            System::

                            getParams()

                                    .maxVisibleFramesPerMarker)
                    ) {

                _12358233879185425501[

                        mf.first]

                        .insert(

                                mf.second.begin(), mf.second.end()
                        );

            } else {

                vector<

                        uint32_t>

                        vframes(
                        mf.second.begin(), mf.second.end()
                );

                pair<

                        size_t, size_t>

                        bestIdx;
                float

                        maxD =
                        std::

                        numeric_limits<
                                float>

                        ::
                        lowest();

                for (

                        size_t i =

                                0; i <

                                   vframes.size();

                        i++

                        ) {

                    for (
                            size_t j =

                                    i + 1;

                            j <

                            vframes.size();

                            j++

                            ) {

                        auto

                                dist =

                                _124580014028079534[

                                        _706246308970949(

                                                vframes[
                                                        i], vframes[

                                                        j]

                                        )
                                ];

                        if

                                (

                                dist >

                                maxD) {

                            bestIdx =

                                    {

                                            vframes[i], vframes[

                                            j]
                                    };

                            maxD =

                                    dist;
                        }

                    }
                }

                _12358233879185425501[

                        mf.first]

                        .insert(bestIdx.first);

                _12358233879185425501[

                        mf.first]

                        .insert(

                                bestIdx.second);

                while (
                        _12358233879185425501[

                                mf.first]

                                .size()

                        <

                        size_t(

                                System::

                                getParams()

                                        .maxVisibleFramesPerMarker)

                        ) {

                    std::

                    pair<

                            uint32_t, float>
                            best(
                            0, std::

                            numeric_limits<

                                    float>

                            ::

                            lowest()
                    );
                    for (

                            size_t i =

                                    0;

                            i < vframes.size();

                            i++

                            ) {

                        if
                                (
                                _12358233879185425501[

                                        mf.first]

                                        .count(
                                                vframes[

                                                        i]

                                        )

                                ==

                                0) {

                            auto d =

                                    _10086624862567280113(

                                            vframes[i], _12358233879185425501[
                                                    mf.first]

                                    );

                            if

                                    (

                                    d >

                                    best.second)

                                best =

                                        {

                                                vframes[
                                                        i], d};

                        }

                    }

                    _12358233879185425501[

                            mf.first]
                            .insert(best.first);

                }

            }

        }

        std::

        set<

                uint32_t>

                _16997237651734773759;

        for (

            auto ms: _12358233879185425501)

            _16997237651734773759.insert(
                    ms.second.begin(), ms.second.end()

            );

        std::
        set<

                uint32_t>

                _16997209188207231919;

        for (

            auto fidx: _46082575804458778)

            if (

                    _16997237651734773759.count(

                            fidx)

                    ==

                    0)

                _16997209188207231919.insert(

                        fidx);

        return
                _16997209188207231919;

    }

    set<

            uint32_t>
    MapManager::

    _5122744303662631154(

            uint32_t keyframe_idx, int

    max) {

        set<uint32_t>

                _632169897324785074;

        if
                (
                TheMap->

                        keyframes.size()

                <

                size_t(

                        System::
                        getParams()

                                .minNumProjPoints))

            return
                    {

                    };

        auto

                _46082575804458778 =
                TheMap->
                        TheKpGraph.getNeighbors(

                        keyframe_idx);

        _46082575804458778.erase(

                0);

        _46082575804458778.erase(

                1);

        for (

            auto ykf: youngKeyFrames)

            _46082575804458778.erase(

                    ykf.first);

        vector<

                pair<

                        float, uint32_t>
        >

                _12397058489822015781;

        int

                _16997202002988998048 =

                System::

                getParams()

                        .minNumProjPoints;

        for (

            auto fidx: _46082575804458778) {

            int

                    nRedundant = 0, nPoints =

                    0;

            auto

                    &frame =

                    TheMap->

                            keyframes[

                            fidx];

            if

                    (

                    frame.isBad()
                    )

                continue;

            for (

                    size_t i =

                            0;

                    i <

                    frame.ids.size(); i++

                    ) {

                if

                        (

                        frame.ids[

                                i]
                        !=

                        std::

                        numeric_limits<

                                uint32_t>
                        ::
                        max()

                        ) {

                    auto

                            &mp =

                            TheMap->

                                    map_points[

                                    frame.ids[

                                            i]

                            ];

                    if
                            (
                            mp.isBad()

                            )

                        continue;

                    nPoints++;

                    int

                            nObs =

                            0;

                    if (

                            mp.getNumOfObservingFrames()

                            >
                            size_t(

                                    _16997202002988998048)

                            ) {

                        for (const auto

                                    &f_i: mp.getObservingFrames()

                                ) {

                            if
                                    (

                                    f_i.first != fidx &&

                                    !

                                            TheMap->

                                                    keyframes[

                                                    f_i.first]

                                                    .isBad()

                                    )

                                if

                                        (

                                        TheMap->
                                                keyframes[

                                                f_i.first]

                                                .und_kpts[

                                                f_i.second]

                                                .octave <=

                                        frame.und_kpts[
                                                i]
                                                .octave) {

                                    nObs++;

                                    if (
                                            nObs >= _16997202002988998048) {

                                        nRedundant++;
                                        break;

                                    }

                                }

                        }

                    }

                }

            }

            float
                    redudantPerc =

                    float(
                            nRedundant)

                    / float(

                            nPoints);

            if (

                    redudantPerc >

                    System::

                    getParams()

                            .KFCulling) {
                _12397058489822015781.push_back(
                        {

                                redudantPerc, fidx}

                );

            }
        }

        if

                (

                _12397058489822015781.size()

                >
                max) {

            std::

            sort(

                    _12397058489822015781.begin(), _12397058489822015781.end(), [

                    ]

                            (
                                    const pair<

                                            float, uint32_t>

                                    &a, const pair<

                                    float, uint32_t>

                                    &b) {

                        return a.first >
                               b.first;

                    }

            );

            _12397058489822015781.resize(

                    max);

        }

        for (

            auto kf_i: _12397058489822015781) {

            _632169897324785074.insert(kf_i.second);

        }

        return

                _632169897324785074;

    }

    vector<uint32_t>MapManager::_8352839093262355382() {
        std::vector<uint32_t> _11398643651601173081;

        for (auto &mp: TheMap->map_points) {
            if(!mp.isStable() && !mp.isBad()) {
                uint32_t obsths = std::min(uint32_t(3), TheMap->keyframes.size());
                if(mp.isStereo())
                    obsths = std::min(uint32_t(2), TheMap->keyframes.size());
                if(mp.getVisibility() < 0.25){
                    mp.setBad(true);
                }

                else if (mp.kfSinceAddition >=1 && mp.getNumOfObservingFrames()<obsths){
                    mp.setBad(true);
                }

                else if (mp.kfSinceAddition >=3){
                    mp.setStable(true);
                }

                if(mp.kfSinceAddition <5) mp.kfSinceAddition++;
            }

            if (mp.isStable()){
                if(mp.getVisibility()< 0.1){
                    mp.setBad(true);
                }
            }


            if (mp.isBad())
                _11398643651601173081.push_back(mp.id);
        }

        return _11398643651601173081;
    }

    bool
    MapManager::

    _668185896188051300(
            const Frame &_16997228172148074791, uint32_t _16940374161587532565) {

        bool
                _46082575734385716 =

                false, _706246335356026 =

                false, _6807035406428482711 =

                false;

        if

                (

                _16997228172148074791.imageParams.isStereoCamera()

                )

            _6807035406428482711 =
                    _11138245882866350888(
                            _16997228172148074791, _16940374161587532565);

        if

                (

                System::

                getParams()

                        .detectKeyPoints)

            _46082575734385716 =

                    _16884568726948844929(

                            _16997228172148074791, _16940374161587532565);

        if

                (

                !
                        _46082575734385716 &&

                System::

                getParams()

                        .detectMarkers)

            _706246335356026 =

                    _5906010176873986459(

                            _16997228172148074791, _16940374161587532565);

        return (

                _46082575734385716 ||

                _706246335356026 ||

                _6807035406428482711);

    }

    bool MapManager::
    _11138245882866350888(

            const Frame &_16997228172148074791, uint32_t

    _16940374161587532565) {

        if
                (

                !

                        _16997228172148074791.imageParams.isStereoCamera()

                )

            return

                    false;

        int

                _13282101351954432384 = 0;

        int _1339477524456856999 =

                0;

        for (

                size_t _2654435874 =

                        0;

                _2654435874 <

                _16997228172148074791.und_kpts.size();

                _2654435874++
                ) {

            if (

                    _16997228172148074791.getDepth(

                            _2654435874)

                    >

                    0 &&
                    _16997228172148074791.imageParams.isClosePoint(

                            _16997228172148074791.getDepth(

                                    _2654435874)

                    )

                    ) {

                if

                        (

                        _16997228172148074791.ids[

                                _2654435874]

                        !=

                        std::
                        numeric_limits<
                                uint32_t>

                        ::

                        max()

                        &&

                        !

                                _16997228172148074791.flags[_2654435874]

                                        .is(

                                                Frame::
                                                FLAG_OUTLIER)

                        )

                    _1339477524456856999++;

                else
                    _13282101351954432384++;
            }
        }

        if

                (

                (
                        _1339477524456856999 <

                        180 * System::

                        getParams()
                                .KFMinConfidence)

                &&

                (

                        _13282101351954432384 >

                        120 * System::

                        getParams()

                                .KFMinConfidence)

                )

            return
                    true;

        else
            return
                    false;

    }

    bool MapManager::

    _5906010176873986459(

            const Frame &_16997228172148074791, uint32_t
    _16940374161587532565) {

        if

                (

                TheMap->

                        map_markers.size()
                ==

                0)

            return false;

        for (

            auto m: _16997228172148074791.markers) {

            if (

                    TheMap->

                            map_markers.count(

                            m.id)

                    ==

                    0) {

                return

                        true;

            }

        }

        for (
            auto m: _16997228172148074791.markers) {
            if

                    (

                    TheMap->

                            map_markers.count(m.id) !=

                    0) {

                if (

                        TheMap->

                                        map_markers.at(

                                        m.id)

                                .pose_g2m.isValid()

                        ==
                        false) {

                    if

                            (

                            (

                                    _16997228172148074791.getMarkerPoseIPPE(

                                                    m.id)

                                            .err_ratio >
                                    System::

                                    getParams()

                                            .aruco_minerrratio_valid) &&

                            System::
                            getParams().aruco_allowOneFrameInitialization) {

                        return

                                true;

                    }

                }

            }

        }

        for (

            auto m: _16997228172148074791.markers) {

            if
                    (
                    TheMap->
                            map_markers.count(

                            m.id)

                    !=

                    0) {

                const auto

                        &Marker =
                        TheMap->

                                map_markers.at(

                                m.id);

                if (

                        Marker.frames.size()

                        >=
                        System::

                        getParams()
                                .maxVisibleFramesPerMarker)

                    continue;

                if (

                        Marker.pose_g2m.isValid()

                        ) {

                    float

                            minDist =
                            std::numeric_limits<

                                    float>
                            ::

                            max();

                    for (
                        auto

                                f: Marker.frames) {

                        float

                                dist =

                                cv::
                                norm(TheMap->

                                        keyframes[

                                             f]

                                             .pose_f2g.getTvec(), _16997228172148074791.pose_f2g.getTvec()

                                );

                        if (

                                dist <
                                minDist)

                            minDist =

                                    dist;

                    }

                    if (

                            minDist >=

                            System::

                            getParams()

                                    .minBaseLine) {

                        return

                                true;

                    }

                }

            }

        }

        if (

                _16997228172148074791.kpts.size()

                !=

                0)

            return

                    false;

        float

                _16940368387347594694 =

                cv::

                norm(

                        _16997228172148074791.pose_f2g.getTvec(), TheMap->

                                keyframes[

                                _16940374161587532565]

                                .pose_f2g.getTvec()
                );

        if

                (

                _16940368387347594694 >
                System::

                getParams()
                        .minBaseLine) {

            return

                    true;

        }

        return
                false;

    }

    bool

    MapManager::

    _16884568726948844929(

            const Frame &_16997228172148074791, uint32_t _16940374161587532565) {

        auto _8222792191690573285 =
                [

                ]

                        (

                                const Frame &_2654435871) {

                    int

                            _2654435879 =

                            0;

                    for (

                            size_t _2654435874 =

                                    0;

                            _2654435874 <

                            _2654435871.ids.size();

                            _2654435874++

                            )

                        if

                                (

                                _2654435871.ids[

                                        _2654435874]
                                !=

                                std::
                                numeric_limits<

                                        uint32_t>::
                                max()

                                )

                            if

                                    (

                                    !

                                            _2654435871.flags[

                                                    _2654435874]

                                                    .is(

                                                            Frame::

                                                            FLAG_OUTLIER)

                                    )

                                _2654435879++;

                    return

                            _2654435879;

                };

        int

                _8367785432631711677 =

                _8222792191690573285(

                        _16997228172148074791);

        if

                (

                _8367785432631711677 <

                20)

            return false;

        float

                _10934236797308178385 =

                System::

                getParams()
                        .KFMinConfidence;

        uint32_t

                _3005399795202072660 =

                3;

        if (
                TheMap->

                        keyframes.size()

                ==

                2) {

            _3005399795202072660 =

                    2;

        }

        int _16937194960156429046 =

                0;

        const auto

                &_3005399819707726498 =

                TheMap->

                        keyframes[_16940374161587532565];

        for (

            auto id: _3005399819707726498.ids) {

            if

                    (id !=

                     std::

                     numeric_limits<
                             uint32_t>

                     ::max()) {

                const auto

                        &mapP =

                        TheMap->

                                map_points[

                                id];

                if (

                        mapP.isBad()

                        )

                    continue;

                if
                        (
                        mapP.getNumOfObservingFrames()

                        <

                        _3005399795202072660)
                    continue;

                _16937194960156429046++;

            }

        }

        if
                (

                _8367785432631711677 <

                float(

                        _16937194960156429046)

                * _10934236797308178385)

            return true;

        return

                false;

    }

    vector<

            uint32_t>

    MapManager::

    _489363023531416435(
            Frame &NewFrame, size_t maxFrames) {

        if
                (
                TheMap->

                        keyframes.size()

                <=

                2)

            return

                    {

                            TheMap->

                                            keyframes.front()

                                    .idx};

        vector<
                uint32_t>

                _1515469360845371082 =

                TheMap->

                        TheKpGraph.getNeighborsV(
                        NewFrame.idx);

        size_t _2654435874 =

                0;

        while (

                _2654435874 <

                _1515469360845371082.size()

                ) {

            if

                    (
                    TheMap->

                            keyframes[
                            _1515469360845371082[
                                    _2654435874]

                    ]

                            .isBad()

                    ) {

                std::
                swap(

                        _1515469360845371082[

                                _2654435874], _1515469360845371082.back()

                );

                _1515469360845371082.pop_back();

            } else

                _2654435874++;

        }

        std::sort(
                _1515469360845371082.begin(), _1515469360845371082.end(), [

                        &]
                        (

                                uint32_t a, uint32_t

                        b) {

                    return
                            TheMap->

                                    TheKpGraph.getWeight(

                                    a, NewFrame.idx)
                            >

                            TheMap->

                                    TheKpGraph.getWeight(

                                    b, NewFrame.idx);
                }

        );

        vector<uint32_t>

                _18082515013534369065;

        for (

            auto neigh: _1515469360845371082) {

            auto
                    medianDepth =

                    TheMap->
                            getFrameMedianDepth(

                            neigh);

            auto baseline =

                    cv::

                    norm(

                            NewFrame.getCameraCenter()

                            - TheMap->

                                    keyframes[

                                    neigh].getCameraCenter()

                    );

            float acos =

                    NewFrame.getCameraDirection()
                            .dot(
                                    TheMap->keyframes[
                                            neigh]
                                            .getCameraDirection()
                            );

            if (

                    acos >

                    0.6 && baseline / medianDepth >

                           System::

                           getParams()

                                   .baseline_medianDepth_ratio_min)

                _18082515013534369065.push_back(

                        neigh);

            if

                    (

                    _18082515013534369065.size()
                    >=

                    maxFrames)

                break;

        }

        return
                _18082515013534369065;

    }

    vector<

            uint32_t>

    MapManager::

    _17400054198872595804(

            Frame &mpCurrentKeyFrame) {

        auto

                _4969073986308462195 =

                TheMap->
                        TheKpGraph.getNeighbors(

                        mpCurrentKeyFrame.idx);

        set<

                uint32_t>

                _8613511226855067609;

        vector<

                uint32_t>
                _18198621160182713342;
        for (

            auto n: _4969073986308462195)

            if

                    (

                    !

                            TheMap->

                                    keyframes[
                                    n]

                                    .isBad()

                    )

                _8613511226855067609.insert(

                        n);

        int

                _1517243165919133649 =

                0, _3005399801165696099 =

                0;

        float

                _175247759809 =

                2.5;

        vector<

                uint32_t>
                _13928263410240979211 =

                mpCurrentKeyFrame.getMapPoints();

        for (

            auto tkf: _8613511226855067609) {

            Frame &keyframe =

                    TheMap->
                            keyframes[

                            tkf];

            cv::

            Point3f camCenter =

                    keyframe.getCameraCenter();

            for (
                auto MpId: _13928263410240979211) {

                if

                        (

                        !

                                TheMap->map_points.is(

                                        MpId)
                        )
                    continue;

                MapPoint &MP =

                        TheMap->

                                map_points[
                                MpId];

                if

                        (

                        MP.isBad()

                        )

                    continue;

                if

                        (

                        MP.frames.count(keyframe.idx)

                        )
                    continue;

                cv::
                Point2f p2d =

                        keyframe.project(

                                MP.getCoordinates(), true, true);

                if (

                        isnan(

                                p2d.x)

                        )

                    continue;

                float

                        dist =

                        cv::

                        norm(

                                camCenter - MP.getCoordinates()

                        );
                if

                        (

                        dist <

                        0.8f * MP.getMinDistanceInvariance()

                        ||

                        dist >

                        1.2f * MP.getMaxDistanceInvariance()

                        )

                    continue;

                if (

                        MP.getViewCos(

                                camCenter)

                        <

                        0.5)

                    continue;

                int

                        nPredictedLevel =

                        mpCurrentKeyFrame.predictScale(

                                dist, MP.getMaxDistanceInvariance()
                        );

                float
                        radius =

                        _175247759809 * keyframe.scaleFactors[

                                nPredictedLevel];

                if
                        (

                        MP.getViewCos(
                                camCenter)

                        <

                        0.98)

                    radius *=
                            1.4f;

                vector<

                        uint32_t>

                        vkpIdx =
                        keyframe.getKeyPointsInRegion(

                                p2d, radius, nPredictedLevel - 1, nPredictedLevel);

                pair<float, int>

                        best(

                        System::

                        getParams()

                                .maxDescDistance + 1e-3, -1);

                for (

                    auto kpidx: vkpIdx) {

                    float

                            descDist =

                            MP.getDescDistance(

                                    keyframe.desc.row(
                                            kpidx)

                            );

                    if

                            (

                            descDist <

                            best.first)

                        best =

                                {
                                        descDist, kpidx};
                }

                if
                        (

                        best.second !=

                        -1) {

                    if

                            (

                            keyframe.ids[

                                    best.second]

                            !=
                            std::

                            numeric_limits<
                                    uint32_t>
                            ::

                            max()
                            ) {

                        TheMap->fuseMapPoints(

                                keyframe.ids[

                                        best.second], MP.id, false);

                        _18198621160182713342.push_back(

                                MP.id);

                        MP.setBad(

                                true);

                        _3005399801165696099++;

                    } else {

                        TheMap->

                                addMapPointObservation(

                                MP.id, keyframe.idx, best.second);

                        _1517243165919133649++;

                    }

                }

            }

        }

        std::

        vector<

                uint32_t>

                _16997228247169055403 = TheMap->

                getMapPointsInFrames(
                _8613511226855067609.begin(), _8613511226855067609.end()

        );

        float

                _13976965695925359212 =
                log(

                        mpCurrentKeyFrame.getScaleFactor()
                );

        cv::

        Point3f _16987816518187263273 =

                mpCurrentKeyFrame.getCameraCenter();
        for (

            auto &mpid: _16997228247169055403) {

            auto

                    &MP =

                    TheMap->

                            map_points[
                            mpid];

            if

                    (

                    MP.isBad()
                    )

                continue;
            if (

                    MP.isObservingFrame(

                            mpCurrentKeyFrame.idx)

                    )
                continue;

            cv::

            Point2f p2d =

                    mpCurrentKeyFrame.project(

                            MP.getCoordinates(), true, true);

            if (isnan(

                    p2d.x)

                    )
                continue;

            float dist =

                    cv::
                    norm(

                            _16987816518187263273 - MP.getCoordinates());
            if

                    (

                    dist <
                    0.8f * MP.getMinDistanceInvariance()

                    ||

                    dist >

                    1.2f * MP.getMaxDistanceInvariance()
                    )
                continue;

            if (

                    MP.getViewCos(_16987816518187263273)

                    <

                    0.5)

                continue;

            int
                    nPredictedLevel =

                    mpCurrentKeyFrame.predictScale(

                            dist, MP.getMaxDistanceInvariance()

                    );

            const float

                    radius =

                    _175247759809 * mpCurrentKeyFrame.scaleFactors[
                            nPredictedLevel];

            vector<
                    uint32_t>

                    vkpIdx =

                    mpCurrentKeyFrame.getKeyPointsInRegion(

                            p2d, radius, nPredictedLevel - 1, nPredictedLevel);

            pair<

                    float, int>

                    best(
                    System::

                    getParams()

                            .maxDescDistance + 1e-3, -1);

            for (

                auto kpidx: vkpIdx) {

                float

                        descDist =

                        MP.getDescDistance(

                                mpCurrentKeyFrame.desc.row(kpidx)
                        );
                if
                        (
                        descDist <

                        best.first)

                    best =

                            {descDist, kpidx};

            }

            if

                    (
                    best.second !=
                    -1) {

                if

                        (

                        mpCurrentKeyFrame.ids[
                                best.second]

                        !=

                        std::
                        numeric_limits<
                                uint32_t>

                        ::
                        max()

                        ) {

                    TheMap->

                            fuseMapPoints(

                            mpCurrentKeyFrame.ids[

                                    best.second], MP.id, false);

                    _18198621160182713342.push_back(

                            MP.id);
                    MP.setBad(
                            true);
                    _3005399801165696099++;

                } else {

                    TheMap->

                            addMapPointObservation(

                            MP.id, mpCurrentKeyFrame.idx, best.second);

                    _1517243165919133649++;

                }

            }
        }

        return

                _18198621160182713342;

    }

    std::

    list<MapManager::

    NewPointInfo>

    MapManager::

    _8820655757626307961(

            Frame &NewFrame) {

        if

                (

                !

                        NewFrame.imageParams.isStereoCamera())

            return

                    {

                    };

        struct

        _14315452481299618814 {

            float

                    _4616368654387135743;

            size_t _5734006271547469041;

            bool

            operator<

                    (
                            const _14315452481299618814 &_175247760080) const {

                return _4616368654387135743 <

                       _175247760080._4616368654387135743;

            }

            bool operator>

                    (

                            const _14315452481299618814 &_175247760080)

            const {
                return _4616368654387135743 >

                       _175247760080._4616368654387135743;

            }

        };

        if (

                System::

                getParams()

                        .KPNonMaximaSuppresion)

            NewFrame.nonMaximaSuppresion();

        vector<

                _14315452481299618814>

                _7619806436859450970;

        _7619806436859450970.reserve(

                NewFrame.ids.size()

        );

        for (
                size_t i = 0; i < NewFrame.ids.size();

                i++

                )

            if
                    (

                    NewFrame.ids[

                            i]

                    ==
                    std::

                    numeric_limits<

                            uint32_t>

                    ::

                    max() &&

                    !
                            NewFrame.flags[

                                    i]
                                    .is(

                                            Frame::

                                            FLAG_NONMAXIMA)

                    &&
                    NewFrame.getDepth(

                            i)

                    >

                    0 &&

                    NewFrame.imageParams.isClosePoint(

                            NewFrame.getDepth(

                                    i)

                    )

                    )

                _7619806436859450970.push_back(

                        {
                                NewFrame.getDepth(

                                        i), i}

                );

        if (
                _7619806436859450970.size()

                >

                tslam::

                System::

                getParams()
                        .maxNewPoints) {

            std::

            random_shuffle(

                    _7619806436859450970.begin(), _7619806436859450970.end()

            );

        }

        _7619806436859450970.resize(

                std::

                min(

                        _7619806436859450970.size(), size_t(

                                tslam::

                                System::

                                getParams()

                                        .maxNewPoints)

                )

        );
        std::

        list<

                MapManager::
                NewPointInfo>

                _4622533121193472218;

        auto

                _16937226146608657651 =

                NewFrame.pose_f2g.inv();

        for (

            auto &kpd: _7619806436859450970) {

            MapManager::

            NewPointInfo mapPoint;

            mapPoint.pose =

                    _16937226146608657651 * NewFrame.get3dStereoPoint(

                            kpd._5734006271547469041);

            mapPoint.frame_kpt.push_back(

                    {

                            NewFrame.idx, kpd._5734006271547469041}
            );
            mapPoint.isStereo =

                    true;

            _4622533121193472218.push_back(mapPoint);

            ;
        }

        return

                _4622533121193472218;

    }

    std::vector<

            MapManager::

            NewPointInfo>
    MapManager::createNewPoints(

            Frame &NewFrame, uint32_t

    nn, uint32_t
            maxPoints) {

        if

                (

                NewFrame.ids.size()
                ==

                0)
            return {

            };

        struct _3005401605294789533 {

            _3005401605294789533(

                    uint32_t _13388472731815556334, uint32_t _1513938270035531338, uint32_t
            _7736357855027240696, cv::

                    Point3f _11093821910177, float

                    _16937031022796222526) {

                _18030119007246525509 =
                        _13388472731815556334;

                _681165095198498101 =

                        _1513938270035531338;

                _10333569979786346575 =
                        _7736357855027240696;

                _16701867013855893038 =

                        _11093821910177;

                _11690406023733055431 =

                        _16937031022796222526;

            }

            uint32_t _18030119007246525509;

            uint32_t

                    _681165095198498101;

            uint32_t

                    _10333569979786346575;

            cv::
            Point3f _16701867013855893038;

            float

                    _11690406023733055431;
        };

        Se3Transform _3005399792197371186 =

                NewFrame.pose_f2g.inv();

        vector<uint32_t>

                _13920901643832806846 =

                _489363023531416435(

                        NewFrame, nn);

        vector<
                vector<
                        _3005401605294789533>

        >

                FrameMatches(

                _13920901643832806846.size());

        FrameMatcher _16937386958649118140;

        _16937386958649118140.setParams(
                NewFrame, FrameMatcher::

                MODE_UNASSIGNED, System::

                                 getParams()
                                         .maxDescDistance * 2, 0.6, true, std::

                numeric_limits<
                        int>

                ::

                max()

        );

#pragma omp parallel for
        for (

                int mf =

                        0;

                mf <
                int(

                        _13920901643832806846.size()

                );

                mf++

                ) {

            Frame &frame2 =

                    TheMap->
                            keyframes[

                            _13920901643832806846[

                                    mf]

                    ];

            cv::
            Mat FQ2T =

                    frame2.pose_f2g * (

                            NewFrame.pose_f2g.inv()
                    );

            vector<

                    cv::

                    DMatch>
                    matches =
                    _16937386958649118140.matchEpipolar(

                            frame2, FrameMatcher::

                            MODE_UNASSIGNED, FQ2T);

            vector<

                    cv::
                    Point3f> p3d =

                    Triangulate(

                            NewFrame, frame2, FQ2T, matches);

            float

                    ratioFactor =
                    1.5f * System::

                    getParams()

                            .scaleFactor;

            for (
                    size_t i =

                            0;

                    i <

                    matches.size(); i++

                    )

                if

                        (

                        !
                                isnan(
                                        p3d[

                                                i]

                                                .x)

                        ) {

                    cv::

                    Point3f p3global =
                            _3005399792197371186 * p3d[

                                    i];

                    float

                            distNF =
                            cv::norm(

                                    p3global - NewFrame.getCameraCenter()
                            );

                    float

                            distF2 = cv::
                    norm(

                            p3global - frame2.getCameraCenter()

                    );

                    if (

                            distNF ==

                            0 ||

                            distF2 ==

                            0)

                        continue;

                    const float

                            ratioDist =
                            distNF / distF2;

                    int

                            oct_NewFrame =

                            NewFrame.und_kpts[
                                    matches[

                                            i]
                                            .trainIdx]

                                    .octave;

                    int
                            oct_frame2 =

                            frame2.und_kpts[

                                    matches[

                                            i]
                                            .queryIdx]

                                    .octave;

                    const float

                            ratioOctave =

                            NewFrame.scaleFactors[
                                    oct_NewFrame]

                            / frame2.scaleFactors[

                                    oct_frame2];

                    if (

                            ratioDist * ratioFactor <

                            ratioOctave || ratioDist >

                                           ratioOctave * ratioFactor)

                        continue;

                    FrameMatches[mf]

                            .push_back(
                                    _3005401605294789533(

                                            uint32_t(

                                                    matches[

                                                            i]

                                                            .trainIdx), frame2.idx, uint32_t(
                                                    matches[

                                                            i]

                                                            .queryIdx), p3global, matches[

                                                    i]

                                                    .distance)

                            );

                }

        }

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

                    return

                            _706246330191125.str();
                };

        std::
        map<

                uint32_t, vector<
                        _3005401605294789533>

        >

                _11350249437170142625;

        for (
                size_t mf =

                        0;

                mf <
                FrameMatches.size();

                mf++)
            for (const auto
                        &match: FrameMatches[

                    mf]

                    )

                _11350249437170142625[

                        match._18030119007246525509]

                        .push_back(
                                match);

        std::
        vector<

                MapManager::

                NewPointInfo>
                _4622533121193472218;

        for (

            auto &kp: _11350249437170142625) {

            MapManager::

            NewPointInfo mapPoint;

            int

                    bestDesc =

                    -1;
            int bestOctave =
                    std::

                    numeric_limits<

                            int>

                    ::

                    max();
            for (

                    size_t di =

                            0;

                    di <

                    kp.second.size();

                    di++

                    ) {

                const auto

                        &frame_kp =
                        TheMap->

                                keyframes[
                                kp.second[

                                        di]

                                        ._681165095198498101]

                                .und_kpts[kp.second[

                                di]

                                ._10333569979786346575];

                if (

                        frame_kp.octave < bestOctave) {
                    bestDesc =

                            di;
                }

            }

            const auto

                    &best_match =

                    kp.second[
                            bestDesc];

            mapPoint.pose =

                    best_match._16701867013855893038;

            mapPoint.dist =
                    best_match._11690406023733055431;

            mapPoint.frame_kpt.push_back(

                    {
                            NewFrame.idx, kp.first}

            );

            for (

                auto fma: kp.second)

                mapPoint.frame_kpt.push_back(
                        {TheMap->

                                keyframes[
                                 fma._681165095198498101]

                                 .idx, fma._10333569979786346575}

                );

            _4622533121193472218.push_back(

                    mapPoint);

            ;

        }

        if (
                _4622533121193472218.size()

                >
                maxPoints) {

            std::

            sort(
                    _4622533121193472218.begin(), _4622533121193472218.end(), [

                    ]

                            (

                                    const MapManager::

                                    NewPointInfo &a, const MapManager::

                            NewPointInfo &b) {
                        return a.dist <

                               b.dist;

                    }

            );

            _4622533121193472218.resize(

                    maxPoints);

        }

        return

                _4622533121193472218;

    }

    void

    MapManager::

    _10758134674558762512(

            int _3005399800582873013) {

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
                _3005399800582873013;

        _3005399798454910266.markersOptWeight =
                System::
                getParams()

                        .markersOptWeight;

        _3005399798454910266.minMarkersForMaxWeight =

                System::

                getParams()
                        .minMarkersForMaxWeight;

        _3005399798454910266.InPlaneMarkers =
                System::

                getParams()

                        .inPlaneMarkers;

        if

                (

                _3005399798454910266.fixed_frames.size()

                ==

                0 &&
                TheMap->

                        map_markers.size()

                ==

                0) {

            auto

                    _175247760151 =
                    TheMap->

                            keyframes.begin();

            _3005399798454910266.fixed_frames.insert(

                    _175247760151->

                            idx);

            ++

                    _175247760151;

            if

                    (

                    _175247760151 !=

                    TheMap->

                            keyframes.end()

                    ) {

                if (
                        _3005399798454910266.used_frames.count(

                                _175247760151->
                                        idx)

                        ||

                        _3005399798454910266.used_frames.size()

                        ==

                        0)

                    _3005399798454910266.fixed_frames.insert(

                            _175247760151->
                                    idx);

            }

        }
        Gopt =

                GlobalOptimizer::

                create(

                        System::

                        getParams()

                                .global_optimizer);

        Gopt->

                setParams(

                TheMap, _3005399798454910266);

        Gopt->

                optimize();

        Gopt->
                getResults(
                TheMap);
        TheMap->
                removeBadAssociations(

                Gopt->

                        getBadAssociations(), System::
                getParams()
                        .minNumProjPoints);

        Gopt =

                nullptr;

    }

    void MapManager::

    _11362629803814604768(

            uint32_t _16937255065087280628, int
    _3005399802176474746) {
        bool
                _16116701644373052209 =

                false;
        for (
            auto _2654435871: TheMap->keyframes)

            if
                    (
                    _2654435871.imageParams.isStereoCamera()

                    ) {

                _16116701644373052209 =

                        true;

                break;

            }
        std::

        set<

                uint32_t>

                _46082575804458778 =

                TheMap->
                        TheKpGraph.getNeighbors(

                        _16937255065087280628, true);

        GlobalOptimizer::

        ParamSet _3005399798454910266(

                debug::

                Debug::
                getLevel()

                >=

                11);

        _3005399798454910266.markersOptWeight =
                System::

                getParams()

                        .markersOptWeight;

        _3005399798454910266.minMarkersForMaxWeight =

                System::
                getParams()

                        .minMarkersForMaxWeight;

        _3005399798454910266.used_frames.insert(

                _46082575804458778.begin(), _46082575804458778.end()

        );

        _3005399798454910266.fixFirstFrame =
                true;

        _3005399798454910266.nIters =

                _3005399802176474746;

        _3005399798454910266.InPlaneMarkers =

                System::
                getParams()

                        .inPlaneMarkers;

        if

                (
                _3005399798454910266.fixed_frames.size()

                ==

                0 &&

                TheMap->

                        map_markers.size()
                ==
                0 &&

                !

                        _16116701644373052209) {

            auto

                    _175247760151 =
                    TheMap->

                            keyframes.begin();

            _3005399798454910266.fixed_frames.insert(
                    _175247760151->

                            idx);

            ++

                    _175247760151;

            if

                    (

                    _175247760151 !=
                    TheMap->

                            keyframes.end()
                    ) {

                if (
                        _3005399798454910266.used_frames.count(

                                _175247760151->

                                        idx)

                        )

                    _3005399798454910266.fixed_frames.insert(
                            _175247760151->

                                    idx);

            }

        }

        Gopt =

                GlobalOptimizer::

                create(

                        System::

                        getParams()

                                .global_optimizer);

        Gopt->

                setParams(

                TheMap, _3005399798454910266);

        Gopt->
                optimize(

                &_hurryUp);

    }

    void

    MapManager::

    toStream(std::

             ostream &_11093822381060) {

        while (

                _curState ==

                WORKING)

            std::

            this_thread::

            sleep_for(std::

                      chrono::

                      milliseconds(

                    10)

            );

        mapUpdate();

        uint64_t _11093822380353 =

                1823312417;

        _11093822381060.write(

                (char *)

                        &_11093822380353, sizeof(

                        _11093822380353)

        );

        _11093822381060.write(
                (

                        char *)

                        &_lastAddedKeyFrame, sizeof(

                        _lastAddedKeyFrame)
        );

        _11093822381060.write((

                                      char *)

                                      &_9728777609121731073, sizeof(

                                      _9728777609121731073)

        );

        _11093822381060.write(

                (

                        char *)

                        &_4090819199315697352, sizeof(
                        _4090819199315697352)

        );

        auto

                _11093821926013 =

                _curState.load();

        _11093822381060.write(

                (

                        char *)

                        &_11093821926013, sizeof(

                        _11093821926013)

        );

        toStream__(

                keyframesToAdd.buffer_, _11093822381060);

        toStream__(

                PointsToRemove, _11093822381060);

        toStream__(

                KeyFramesToRemove, _11093822381060);

        toStream__kv(

                youngKeyFrames, _11093822381060);

        _11093822381060.write(
                (

                        char *)

                        &_hasMapBeenScaled, sizeof(_hasMapBeenScaled)

        );

        _lastAddedKFPose.toStream(

                _11093822381060);

        _11093822381060.write(

                (

                        char *)

                        &bigChangeHasHappen, sizeof(

                        bigChangeHasHappen)
        );

        _11093822381060.write(

                (

                        char *)

                        &_CurkeyFrame, sizeof(
                        _CurkeyFrame)
        );

        _11093822381060.write(

                (

                        char *)

                        &_12303014364795142948, sizeof(

                        _12303014364795142948)

        );

        _11093822381060.write(
                (
                        char *)

                        &_hurryUp, sizeof(

                        _hurryUp)

        );

        _LoopClosureInfo.toStream(

                _11093822381060);

    }

    void

    MapManager::

    fromStream(

            std::

            istream &_11093822381060) {

        stop();

        uint64_t _11093822380353;

        _11093822381060.read(

                (

                        char *)

                        &_11093822380353, sizeof(

                        _11093822380353)

        );
        if (
                _11093822380353 != 1823312417)

            throw std::
            runtime_error(
                    string(
                            __PRETTY_FUNCTION__)

                    +
                    "\x43\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x72\x65\x61\x64\x20\x73\x69\x67\x6e\x61\x74\x75\x72\x65\x20\x6f\x66\x20\x4d\x61\x70\x6d\x61\x6e\x61\x67\x65\x72\x20\x69\x6e\x20\x73\x74\x72\x65\x61\x6d");

        _11093822381060.read(

                (

                        char *)

                        &_lastAddedKeyFrame, sizeof(

                        _lastAddedKeyFrame)

        );
        _11093822381060.read(
                (

                        char *)

                        &_9728777609121731073, sizeof(
                        _9728777609121731073)

        );

        _11093822381060.read(

                (

                        char *)

                        &_4090819199315697352, sizeof(

                        _4090819199315697352)

        );

        auto

                _16987968640077875288 =

                _curState.load();

        _11093822381060.read(
                (

                        char *)

                        &_16987968640077875288, sizeof(_16987968640077875288)

        );

        _curState =

                _16987968640077875288;

        fromStream__(
                keyframesToAdd.buffer_, _11093822381060);

        fromStream__(

                PointsToRemove, _11093822381060);

        fromStream__(
                KeyFramesToRemove, _11093822381060);

        fromStream__kv(
                youngKeyFrames, _11093822381060);

        _11093822381060.read(

                (

                        char *)

                        &_hasMapBeenScaled, sizeof(

                        _hasMapBeenScaled)

        );

        _lastAddedKFPose.fromStream(

                _11093822381060);

        _11093822381060.read(

                (

                        char *)
                        &bigChangeHasHappen, sizeof(
                        bigChangeHasHappen)

        );

        _11093822381060.read(

                (
                        char *) &_CurkeyFrame, sizeof(

                        _CurkeyFrame)

        );

        _11093822381060.read(
                (

                        char *)

                        &_12303014364795142948, sizeof(
                        _12303014364795142948)
        );

        _11093822381060.read(

                (

                        char *)
                        &_hurryUp, sizeof(

                        _hurryUp)

        );

        _LoopClosureInfo.fromStream(

                _11093822381060);

    }

    uint64_t MapManager::getSignature() {

        Hash _11093822380353;

        _11093822380353 +=
                _lastAddedKeyFrame;

        _11093822380353 +=

                _9728777609121731073;

        _11093822380353 +=
                _4090819199315697352;

        _11093822380353 +=

                _curState.load();

        _11093822380353 +=

                keyframesToAdd.size();

        for (

            auto kv: PointsToRemove)

            _11093822380353 +=

                    kv;

        for (

            auto kv: KeyFramesToRemove)

            _11093822380353 +=

                    kv;

        for (

            auto kv: youngKeyFrames) {
            _11093822380353 +=
                    kv.first;

            _11093822380353 +=

                    kv.second;

        }

        _11093822380353 +=

                _hasMapBeenScaled;

        _11093822380353 +=

                _lastAddedKFPose;

        _11093822380353 +=

                bigChangeHasHappen;

        _11093822380353 +=

                _CurkeyFrame;

        _11093822380353 +=
                _12303014364795142948;

        _11093822380353 +=
                _LoopClosureInfo.getSignature();

        return

                _11093822380353;

    }

void MapManager:: loopClosurePostProcessing(

            Frame &_6807141023702418932, const LoopDetector::

    LoopClosureInfo &_11093822343890) {

        auto

                _46082543279161383 =

                []

                        (
                                const vector<

                                        uint32_t>

                                &_2654435887) {

                    std::

                    set<
                            uint32_t>

                            _2654435884;

                    for (

                        auto e: _2654435887)

                        _2654435884.insert(

                                e);

                    return
                            _2654435884;

                };

        auto

                _5232059496476615978 =

                TheMap->

                        TheKpGraph.getNeighborsV(

                        _11093822343890.matchingFrameIdx, true);

        auto

                _5232059496475995487 =

                TheMap->
                        TheKpGraph.getNeighborsV(

                        _11093822343890.curRefFrame, true);
        auto

                &_16935669825082873233 =

                _6807141023702418932;

        if

                (

                !

                        TheMap->
                                keyframes.is(

                                _6807141023702418932.idx)
                ) {

            _16935669825082873233.pose_f2g =

                    _11093822343890.expectedPos;

            _16935669825082873233 =

                    addKeyFrame(
                            &_6807141023702418932);

        }

        int

                _706246332364647 =

                0;

        for (
            auto match: _11093822343890.map_matches) {

            if (_16935669825082873233.ids[

                        match.queryIdx]
                ==

                std::
                numeric_limits<

                        uint32_t>

                ::

                max()

                &&

                !

                        TheMap->

                                map_points[match.trainIdx]

                                .isObservingFrame(

                                        _16935669825082873233.idx)

                    ) {

                TheMap->

                        addMapPointObservation(

                        match.trainIdx, _16935669825082873233.idx, match.queryIdx);

                _706246332364647++;

            }

        }
        _10758134674558762512(

                20);

        auto
                _16937290651980367310 =

                TheMap->

                        TheKpGraph.getNeighborsV(

                        _16935669825082873233.idx, true);

        vector<

                float>

                _16988745808691518194 =

                {

                        4, 2.5};

        for (

                size_t nt =

                        0;
                nt <

                _16988745808691518194.size(); nt++

                ) {

            _706246332364647 =

                    0;

            int

                    nFusions =

                    0;

            for (

                auto fidx: _16937290651980367310) {

                auto

                        &CurFrame =

                        TheMap->

                                keyframes[

                                fidx];

                auto frameMapPoints =

                        _46082543279161383(
                                CurFrame.getMapPoints()

                        );

                auto

                        map_matches =

                        TheMap->

                                matchFrameToMapPoints(

                                _16937290651980367310, CurFrame,
                                CurFrame.pose_f2g, System::

                                                   getParams()
                                                           .maxDescDistance * 2, _16988745808691518194[

                                        nt],
                                false, true, frameMapPoints);

                for (

                    auto match: map_matches) {

                    if (

                            CurFrame.ids[

                                    match.queryIdx]

                            !=

                            std::numeric_limits<

                                    uint32_t>
                            ::
                            max()

                            ) {

                        TheMap->

                                fuseMapPoints(

                                match.trainIdx, CurFrame.ids[

                                        match.queryIdx], true);

                        nFusions++;

                    } else {
                        TheMap->

                                addMapPointObservation(
                                match.trainIdx, CurFrame.idx, match.queryIdx);

                        _706246332364647++;

                    }

                }

            }

            if (

                    _706246332364647 >

                    0 ||
                    nFusions >

                    0) {

                _11362629803814604768(
                        _16935669825082873233.idx, 20);

                TheMap->

                        removeBadAssociations(
                        Gopt->

                                getBadAssociations(), System::

                        getParams().minNumProjPoints);

            }

        }

        _6807141023702418932.pose_f2g =

                _16935669825082873233.pose_f2g;
        _6807141023702418932.ids = _16935669825082873233.ids;

    }

}