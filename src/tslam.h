#ifndef __TSLAM_H__
#define __TSLAM_H__
#include <memory>
#include <opencv2/core/core.hpp>
#include "tslam_exports.h"
#include "tslamtypes.h"
#include "imageparams.h"
#include "map.h"
#include "reconstruction/tslam_reconstructor.hh"

namespace tslam{
    class TSLAM_API TSlam
    {
    public:
        TSlam();
        ~TSlam();

        /**
         * @brief setParams set the required params before processing images
         * @param map a pointer to the map being used. It can be an empty map or an already created one
         * @param params controlling the system behaviuor
         * @param vocabulary optional path to the dictionary BOW. Without it, relocalization is not yet possible.
         */
        void setParams(std::shared_ptr<Map> map, const  tslam::Params &params, const std::string &vocabulary="");

        /// Clear this object setting it to intial state. It will remove all map data
        void clear();

        /// returns system params
        static Params &getParams() ;


        /// Feeds the system with a new image and the parameters of the camera it has been processed with. The final
        /// parameter is the index of the sequence
        /// Returns the camera pose estimated. The pose is the transform moving points from the global reference sytem to the camera reference ssytem
        cv::Mat process( cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx);

        /// Reset the current frame pose. Use it to start tracking in a known map
        void resetTracker();

        /// sets the system mode
        void setMode(MODES mode);

        /// sets the system in lost mode
        //void resetCurrentPose();

        /// returns the number of the last processed framed
        uint32_t getLastProcessedFrame()const;


        /// Saves the current state of the system to a file so that it can be recovered later. It is only safe if the system is in sequential mode
        void saveToFile(std::string filepath);
        /// Loads the state of the system from a file
        void readFromFile(std::string filepath);


        /// only for debugging pourposes perform the global optimization
        void globalOptimization();

        /// waits for all threads to finish
        void waitForFinished();


        /// returns the index of the current keyframe
        uint32_t getCurrentKeyFrameIndex();


        /// returns a pointer to the map being used
        inline std::shared_ptr<Map> getMap() { return map; } ;


        /// returns an string that identifies the system state. It is like a md5 sum the system state
        std::string getSignatureStr()const;


        void setDebugLevel(int level);
        void showTimers(bool v);

        /// will update the internal parameters.Not all parameters can be changed
        void updateParams(const Params &p);

        /**
        * Things the original UcoSlam doesn't implement
        **/
        std::shared_ptr<tslam::Map> map;
        tslam::Params systemParams;
        tslam::ImageParams imageParams;

        /**
         * @brief Clear the map.
         */
        void clearMap();
        /**
         * @brief Set the map
         * @param pathToMap path to the .map file
         */
        void setMap(std::string pathToMapFile, bool updateSystemImmediately=false);
        /**
         * @brief Set the vocabulary
         * @param pathToMap path to the .fbow file
         */
        void setVocabulary(std::string pathToVocFile);
        /**
         * @brief Set the camera parameter
         * @param pathToMap Path to the camera parameter file, the structure should be like
         */
        void setCamParams(std::string pathToCamParamFile);
        /**
         * Set the status of the map
         *
         * Set the status of the map. If "localizeOnly" is set to true global optimization will be off and the
         * new added keyframes will be kept in a fixed number
         * @param localizeOnly true/false
         */
        void setInstancing(bool localizeOnly);

        /**
         * @brief Get the last tracked camera pose (not guarentee to be the last frame)
         * @return A 4x4 cv::Mat; cv::Mat::eye if not tracked ever.
         */
        inline cv::Mat getLastTrackedCamPose() { return lastTrackedCamPose; };
        /**
         * @brief get the camera pose of the last processed frame,
         * @return A 4x4 cv::Mat; cv::eye if last frame was not tracked.
         */
//        inline cv::Mat getLastCamPose(){ return lastCamPose; };
        /**
         * @brief Process a frame
         * @param frame Frame to process
         * @param camPose The reference will be updated to the camera pose of the frame.
         * @return A boolean indicate if is tracked
         */
        bool process(cv::Mat frame, cv::Mat &camPose);

    private:
        void *impl;
        void updateSystem();

        std::string pathToVoc = "";
        std::string pathToMap = "";
        std::string pathToCamParam = "";

        // cv::Mat lastCamPose = cv::Mat::eye(4, 4, CV_32F);
        cv::Mat lastTrackedCamPose = cv::Mat::eye(4, 4, CV_32F);

        int currentFrameIndex = 0;

        bool toUpdateSystem = false;
        bool toClearMap = false, toResetMap = false;
        bool toResetCamParams = false;

    // static functions
    public:
        /**
         * Combine two map (by aligning the tags)
         * @param mapPathA path to the first map
         * @param mapPathB path to the second map
         * @param outputPath path to the combined map
         * @param exportYml if export yml alongside
         * @param exportPly if export ply alongside
         * @param estimatedImageParam pointer to the estimated image parameter
         * @param niters how many iterations to run during optimization
         */
        static void CombineMap(string mapPathA, string mapPathB, string outputPath, bool exportYml=true, bool exportPly=true, ImageParams *estimatedImageParam=nullptr, int niters=50);
        /**
         * @brief Reconstruct the 3D model from the current mapping and export the .ply mesh
         * @param filepath output path of the exported .ply
         * @return true if succeed, false otherwise
         */
        static bool Reconstruct3DModelAndExportPly(const std::string filepath);
    };

}
#endif
