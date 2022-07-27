#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include "featureextractors/feature2dserializable.h"
#include "map_types/frame.h"
#include "imageparams.h"
#include "utils/markerdetector.h"
#include <memory>
#include "utils/mapinitializer.h"

namespace aruco {
  class MarkerDetector ;
};

namespace ucoslam {
  class FrameExtractor {

public:
    FrameExtractor ()

  ;

     void

 setParams (

 std ::
     shared_ptr <

    Feature2DSerializable >

       feature_detector, const ucoslam ::

 Params &params, std ::

    shared_ptr <

ucoslam ::

 MarkerDetector > mdetector )
    ;

     void

   process (

 const cv ::
  Mat &image, const ImageParams &ip,Frame &frame, uint32_t
   frameseq_idx =

     std ::
    numeric_limits <

  uint32_t > ::

max ()
    )

    ;

     void

     process_rgbd (

  const cv :: Mat &image, const cv ::

  Mat &depthImage,const ImageParams &ip,Frame &frame, uint32_t

    frameseq_idx =

 std ::

     numeric_limits <

   uint32_t >
        ::
     max () )

  ;
     void

    processStereo (

 const cv ::

    Mat &LeftRect, const cv ::

 Mat &RightRect,const ImageParams &ip,Frame &frame, uint32_t
       frameseq_idx =

 std ::
  numeric_limits <

    uint32_t >
 :: max ()

    )

     ;

     void

processArray (

const vector <

     cv ::
 Mat >

   &images, ImageParams &ArrayCamParams , Frame &frame, uint32_t

   frameseq_idx, const std ::

   shared_ptr <

MapInitializer >

 map_init )
      ;

     void

    getMatches (
std ::

 vector <

  cv :: DMatch >
       & matches, const Frame &frame, const std ::
    vector <
 cv ::

         KeyPoint > &trainKpts,
                                    const cv ::
 Mat& Fund, const cv ::

         Mat &trainDesc, int

    t =
 0 )

 ;

     bool

       &removeFromMarkers ()

       {
return _12350051723532614025 ;

       }

     bool

   &detectMarkers ()

    {

   return _3566717627060593117 ;

 }

     bool &detectKeyPoints ()

    {

   return _4309197024622458338 ;

        }

      void

    toStream (

      std ::

   ostream &str )

    const ;

     void

fromStream ( std ::

  istream &str )
      ;

     void

 setSensitivity (

   float v ) ;

     float

          getSensitivity ()

      ;

private:

     struct
 ImgInfo {

        cv ::
Mat _15530082771795719302,_8358128829407646415 ;

        ImageParams _15530082765074651952,_5505640830793117477 ;
        pair <
   float,float >

          _6972553715263421613 ;

     }

     ;

    std ::
   shared_ptr <

      Feature2DSerializable > _8033463663468506753 ;

     void

       _14329080428242784455 (
      float _17370277987955713200,const cv ::

  Mat &_11093822404769,const ImageParams &_175247760147,const cv ::

Mat &_11093822404770 =
cv :: Mat ()

      )

    ;

     void

  _10230054520346001887 (

   float _17370277987955713200,const std ::

    vector <

 cv ::

Mat >

 &_3005401535270843804, const ImageParams &_175247760147 )
       ;

  void extractFrame (const ImgInfo& Iinfo, Frame &frame, uint32_t frameseq_idx =

 std ::
numeric_limits <

     uint32_t >

  ::
     max ()
   )

        ;

     vector <

    ImgInfo >

 _12800511165451773841 ;

     uint32_t

        _12273370977065616393 =

    0 ;

     bool

   _12350051723532614025 =

 true ;

     bool

  _3566717627060593117 =

    true ;

     bool

     _4309197024622458338 = true ;

    Feature2DSerializable :: FeatParams _10675870925382111478 ;

    std ::
  shared_ptr <
    ucoslam ::
 MarkerDetector >

     _8000946946827829134 ;

     float
    _12693418215446769236 =

  0 ;

     float

       _13206983270957238750 =

  0 ;

    ucoslam :: Params _13116459431724108758 ;

 }

 ;

 }