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


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"
#include "Tracking.h"
#include "GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class Tracking;

class GeometricCamera;

class KeyFrame : public enable_shared_from_this<KeyFrame>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB, Tracking* pTracking);

    // Pose functions
    void SetPose(const Sophus::SE3f &Tcw);
    void SetVelocity(const Eigen::Vector3f &Vw_);

    Sophus::SE3f GetPose();

    Sophus::SE3f GetPoseInverse();
    Eigen::Vector3f GetCameraCenter();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(shared_ptr<KeyFrame> pKF, const int &weight);
    void EraseConnection(shared_ptr<KeyFrame> pKF);

    void UpdateConnections(bool upParent=true);
    void UpdateBestCovisibles();
    std::set<shared_ptr<KeyFrame>> GetConnectedKeyFrames();
    std::vector<shared_ptr<KeyFrame> > GetVectorCovisibleKeyFrames();
    std::vector<shared_ptr<KeyFrame>> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<shared_ptr<KeyFrame>> GetCovisiblesByWeight(const int &w);
    int GetWeight(shared_ptr<KeyFrame> pKF);

    // Spanning tree functions
    void AddChild(shared_ptr<KeyFrame> pKF);
    void EraseChild(shared_ptr<KeyFrame> pKF);
    void ChangeParent(shared_ptr<KeyFrame> pKF);
    std::set<shared_ptr<KeyFrame>> GetChilds();
    shared_ptr<KeyFrame> GetParent();
    bool hasChild(shared_ptr<KeyFrame> pKF);
    void SetFirstConnection(bool bFirst);

    // Loop Edges
    void AddLoopEdge(shared_ptr<KeyFrame> pKF);
    std::set<shared_ptr<KeyFrame>> GetLoopEdges();

    // Merge Edges
    void AddMergeEdge(shared_ptr<KeyFrame> pKF);
    set<shared_ptr<KeyFrame>> GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(shared_ptr<MapPoint> pMP, const size_t &idx);
    void EraseMapPointMatch(const int &idx);
    cv::Mat GetDescriptor(const int &idx);
    void EraseBadDescriptor();
    void EraseMapPointMatch(shared_ptr<MapPoint> pMP);
    void ReplaceMapPointMatch(const int &idx, shared_ptr<MapPoint> pMP);
    std::set<shared_ptr<MapPoint>> GetMapPoints();
    std::vector<shared_ptr<MapPoint>> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    shared_ptr<MapPoint> GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false);
    bool UnprojectStereo(int i, Eigen::Vector3f &x3D);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(shared_ptr<KeyFrame> pKF1, shared_ptr<KeyFrame> pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void SetNewBias(const IMU::Bias &b);
    Eigen::Vector3f GetGyroBias();

    Eigen::Vector3f GetAccBias();

    IMU::Bias GetImuBias();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    std::vector< std::vector <std::vector<size_t> > > GetFeatureGrids(){
        return mGrid;
    }

    bool bImu;

    bool UpdateCountInLocalMapping(bool bLocal);
    bool UpdateCountInTracking(bool bLocal);
    bool isNonLocal();
    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnMapSaprsificationId;
    long unsigned int mnBAFixedForKF;

    //Number of optimizations by BA(amount of iterations in BA)
    long unsigned int mnNumberOfOpt;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    bool mbCurrentPlaceRecognition;

    // Variables used by loop closing
    Sophus::SE3f mTcwGBA;
    Sophus::SE3f mTcwBefGBA;
    Eigen::Vector3f mVwbGBA;
    Eigen::Vector3f mVwbBefGBA;
    IMU::Bias mBiasGBA;
    long unsigned int mnBAGlobalForKF;

    // Variables used by merging
    Sophus::SE3f mTcwMerge;
    Sophus::SE3f mTcwBefMerge;
    Sophus::SE3f mTwcBefMerge;
    Eigen::Vector3f mVwbMerge;
    Eigen::Vector3f mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;

    // Pose relative to parent (this is computed when bad flag is activated)
    Sophus::SE3f mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    // Preintegrated IMU measurements from previous keyframe
    shared_ptr<KeyFrame> mPrevKF;
    shared_ptr<KeyFrame> mNextKF;

    shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    IMU::Calib mImuCalib;

    unsigned int mnOriginMapId;

    string mNameFile;

    int mnDataset;

    std::vector <shared_ptr<KeyFrame>> mvpLoopCandKFs;
    std::vector <shared_ptr<KeyFrame>> mvpMergeCandKFs;

    bool mbSparsified;

    static int mnNonLocalKF;
    // The following variables need to be accessed through a mutex to be thread safe.
protected:
    // sophus poses
    Sophus::SE3<float> mTcw;
    Eigen::Matrix3f mRcw;
    Sophus::SE3<float> mTwc;
    Eigen::Matrix3f mRwc;

    // IMU position
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    //Transformation matrix between cameras in stereo fisheye
    Sophus::SE3<float> mTlr;
    Sophus::SE3<float> mTrl;

    // Imu bias
    IMU::Bias mImuBias;

    cv::Mat mDescriptors;
    // MapPoints associated to keypoints
    std::vector<shared_ptr<MapPoint>> mvpMapPoints;
    // For save relation without pointer, this is necessary for save/load function
    std::vector<long long int> mvBackupMapPointsId;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<shared_ptr<KeyFrame>,int> mConnectedKeyFrameWeights;
    std::vector<shared_ptr<KeyFrame>> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    shared_ptr<KeyFrame> mpParent;
    std::set<shared_ptr<KeyFrame>> mspChildrens;
    std::set<shared_ptr<KeyFrame>> mspLoopEdges;
    std::set<shared_ptr<KeyFrame>> mspMergeEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    Map* mpMap;
    Tracking* mpTracking;
    // Calibration
    Eigen::Matrix3f mK_;

    // Mutex
    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;
    std::mutex mMutexSparsify;

    int mnCountInLocal;
    bool mbNonLocalKF;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysRight;
    std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth;
    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;
    // Number of KeyPoints
    int N;
    int NLeft, NRight;
public:
    cv::KeyPoint GetKeyUn(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        return mvKeysUn[idx];
    }
    cv::KeyPoint GetKey(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        return mvKeys[idx];
    }
    cv::KeyPoint GetKeyRight(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        return mvKeysRight[idx];
    }
    float GetuRight(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        return mvuRight[idx];
    }
    float GetDepth(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        return mvuRight[idx];
    }
    cv::KeyPoint GetKeyPoint(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        if(NLeft == -1)
            return mvKeysUn[idx];
        else if(idx < NLeft)
            return mvKeys[idx];
        else
            return mvKeysRight[idx - NLeft];
    }
    int GetScaleLevel(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        if(NLeft == -1)
            return mvKeysUn[idx].octave;
        else if(idx < NLeft)
            return mvKeys[idx].octave;
        else
            return mvKeysRight[idx - NLeft].octave;
    }
    bool FromRightImage(size_t idx){
        unique_lock<mutex> lock(mMutexSparsify);
        if(NLeft == -1 || idx < NLeft)
            return false;
        else
            return true;
    }
    int GetN(){
        unique_lock<mutex> lock(mMutexSparsify);
        return N;
    }
    int GetNLeft(){
        unique_lock<mutex> lock(mMutexSparsify);
        return NLeft;
    }
    std::vector<cv::KeyPoint> GetAllKeyUn();
    DBoW2::BowVector GetBowVector();
    DBoW2::FeatureVector GetFeatureVector();

    GeometricCamera* mpCamera, *mpCamera2;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    Sophus::SE3f GetRelativePoseTrl();

    //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
//    std::vector<cv::KeyPoint> mvKeysRight;

    std::vector< std::vector <std::vector<size_t> > > mGridRight;

    Sophus::SE3<float> GetRightPose();
    Sophus::SE3<float> GetRightPoseInverse();

    Eigen::Vector3f GetRightCameraCenter();
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
