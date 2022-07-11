#include <memory>
#include <mutex>
#include <thread>
#include <mutex>
#include <atomic>
#include "basictypes/tsqueue.h"
#include "utils/loopdetector.h"

# 1 "/app/example.cpp"
# 1 "/app//"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "/app/example.cpp"
# 637 "/app/example.cpp"
 namespace

      ucoslam {

 class

     Map ;

 class

    GlobalOptimizer ;

 class

    MapManager {

 struct
NewPointInfo {

    cv ::
 Point3d pose ;

     bool

isStereo =
     false ;

    vector < pair <

       uint32_t,uint32_t >
        >

   frame_kpt ;

     float

   dist =
    std ::

numeric_limits <

float >

  ::

 max ()

      ;

 }

 ;

public:
    enum STATE {
      IDLE,WORKING,WAITINGFORUPDATE }

 ;

    MapManager ()

    ;

     ~
      MapManager ()

      ;

     void
setParams (

 std ::

  shared_ptr <

Map >
   map, bool
   EnableLoopClosure )

 ;

     bool

hasMap ()
    const ;

      void

    start ()
      ;

     void

        stop ()

     ;

     void

 reset ()

 ;

     int

    newFrame (

Frame &kf, int32_t curkeyFrame )

     ;

     bool

   mapUpdate (

     void )

       ;

     bool
     bigChange ()

 const ;

    Se3Transform getLastAddedKFPose ()
   ;

     uint32_t

getLastAddedKeyFrameIdx ()
      const ;

     void

        toStream (
   std ::
 ostream &str )
        ;

     void

      fromStream (

 istream &str )

     ;

    uint64_t getSignature ()

 ;
private:

     void

_8669746328630631075 () ;

     void

    _12295639104386009589 ()

  ;

    Frame &_1018502486064296669 (
   Frame *_2654435871 )
      ;

     bool

     _668185896188051300 (
 const Frame &_16997228172148074791 , uint32_t

_16940374161587532565 )

      ;

     bool
     _5906010176873986459 (

    const Frame & _16997228172148074791 , uint32_t

  _16940374161587532565 )
        ;

     bool

     _16884568726948844929 (
  const Frame & _16997228172148074791, uint32_t

  _16940374161587532565 )

 ;

     bool

         _11138245882866350888 (
    const Frame &_16997228172148074791, uint32_t

_16940374161587532565 )

     ;

    std ::

    vector <

   NewPointInfo >

     _13988982604287804007 (

   Frame &_16935669825082873233 , uint32_t

 _175247759447 =

    20, uint32_t

  _1522768807369958241 =

  std ::

     numeric_limits <
       uint32_t >

         ::

    max ()

  )

     ;

    std ::

     list <

     NewPointInfo >

  _8820655757626307961 (

   Frame & _16937201236903537060 )
      ;

    vector <
 uint32_t >

  _8352839093262355382 () ;

     void

  _11362629803814604768 (

      uint32_t _16937255065087280628, int

     _3005399802176474746 =
    5 )
 ;

     void

       _10758134674558762512 (

        int _3005399800582873013 =

  10 )

  ;

    set <
   uint32_t >

   _12040998890815479673 (

    uint32_t _13776525365726664701 )

    ;

    set < uint32_t >
    _17920146964341780569 (

    uint32_t _13776525365726664701 )
 ;

    set < uint32_t > _5122744303662631154 (
       uint32_t _13776525365726664701, int

 _11093822290295 =

   1 )

         ;

    vector <
      uint32_t >
    _489363023531416435 (
   Frame &_16935669825082873233, size_t _2654435879 )

        ;

     void

     _12244964123780599670 (

 Frame &_46082543180066935, const LoopDetector ::

LoopClosureInfo &_11093822343890 )

    ;

    vector < uint32_t >

        _17400054198872595804 (

     Frame &_9083779410036955469 )
 ;

    std ::

     shared_ptr <

BaseLoopDetector >
        _4644858540263212367 (

     bool _18278402211387234209 )

         ;

    std ::

   thread _4098354751575524583 ;

    std ::

    mutex _5496678766866853603 ;

    TSQueue <

Frame* >

  _5860250156218117893 ;

    std ::

  shared_ptr <
      Map >

    _3370013330161650239 ;

     bool

  _12303014364795142948 =
   false ;

     int

 _9728777609121731073 =

  0 ;

    std ::

  atomic <

   STATE >

 _9129579858736004991 ;

     bool
       _4090819199315697352 =

      false ;

      uint32_t

 _11028815416989897150 =
std ::

    numeric_limits <

      uint32_t > ::

      max ()
   ;

    vector <

 uint32_t >

   _7124056634192091721 ;

    set <

 uint32_t >

     _2225497823225366210 ;

    std :: shared_ptr < GlobalOptimizer >
    _15944432432468226297 ;

    std :: map <

   uint32_t, uint32_t

  >

   _15327812228135655144 ;

    Se3Transform _13909239728712143806 ;

     bool
    _1061304613240460439 =

 false ;

    std ::

   shared_ptr <

  BaseLoopDetector >
      _14139181480504378433 ;

    LoopDetector ::

        LoopClosureInfo _8346364136266015358 ;

     uint32_t
  _5097784010653838202 =

   std ::

  numeric_limits <
uint32_t >

 ::
max ()

   ;

     bool

   _13990461397173511559 =
    false ;

     bool
  _4098394392539754261 =
       false ;

     bool

 _10046998516135983211 =
      true ;

 }
       ;

 }