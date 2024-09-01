/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapPoint : public enable_shared_from_this<MapPoint>
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();

    MapPoint(const Eigen::Vector3f &Pos, shared_ptr<KeyFrame> pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, shared_ptr<KeyFrame> pRefKF, shared_ptr<KeyFrame> pHostKF, Map* pMap);
    MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPos();

    Eigen::Vector3f GetNormal();
    void SetNormalVector(const Eigen::Vector3f& normal);

    shared_ptr<KeyFrame> GetReferenceKeyFrame();

    std::map<shared_ptr<KeyFrame>,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(shared_ptr<KeyFrame> pKF,int idx);
    void UpdateObservation(shared_ptr<KeyFrame> pKF,int idx);
    void EraseObservation(shared_ptr<KeyFrame> pKF);

    std::tuple<int,int> GetIndexInKeyFrame(shared_ptr<KeyFrame> pKF);
    bool IsInKeyFrame(shared_ptr<KeyFrame> pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(shared_ptr<MapPoint> pMP);
    shared_ptr<MapPoint> GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, shared_ptr<KeyFrame> pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnMapSparsificationId;
    long unsigned int mnFuseCandidateForKF;
    long unsigned int mnUpdateChi2ForKF;

    long unsigned int mnIndexForSparsification;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    Eigen::Vector3f mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Eigen::Vector3f mPosMerge;
    Eigen::Vector3f mNormalVectorMerge;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

    int mnOptimizedTimesInLBA;
    bool mbSparsified;
    bool mbOptimizedInLocalBA;

protected:

     // Position in absolute coordinates
     Eigen::Vector3f mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<shared_ptr<KeyFrame>,std::tuple<int,int> > mObservations;
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     shared_ptr<KeyFrame> mpRefKF;
     long unsigned int mBackupRefKFId;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     shared_ptr<MapPoint> mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     // Mutex
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
