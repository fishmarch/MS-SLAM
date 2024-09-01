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


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(shared_ptr<KeyFrame> pKF);
    void AddMapPoint(shared_ptr<MapPoint> pMP);
    void AddSparsifiedMapPoint(shared_ptr<MapPoint> pMP);
    void AddSparsifiedKeyFrame(shared_ptr<KeyFrame> pKF);
    void EraseMapPoint(shared_ptr<MapPoint> pMP);
    void EraseKeyFrame(shared_ptr<KeyFrame> pKF);
    void SetReferenceMapPoints(const std::vector<shared_ptr<MapPoint>> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<shared_ptr<KeyFrame>> GetAllKeyFrames();
    std::vector<shared_ptr<KeyFrame>> GetAllSparsifiedKeyFrames();
    std::vector<shared_ptr<MapPoint>> GetAllMapPoints();
    std::vector<shared_ptr<MapPoint>> GetAllSparsifiedMapPoints();
    std::vector<shared_ptr<MapPoint>> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned int SparsifiedMapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    shared_ptr<KeyFrame> GetOriginKF();
    shared_ptr<KeyFrame> GetNewestKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    vector<shared_ptr<KeyFrame>> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    shared_ptr<KeyFrame> mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

    long unsigned int mnId;

    std::set<shared_ptr<MapPoint>> mspMapPoints;
    std::set<shared_ptr<KeyFrame>> mspKeyFrames;

    std::set<shared_ptr<MapPoint>> mspSparsifiedMapPoints;
    std::set<shared_ptr<KeyFrame>> mspSparsifiedKeyFrames;

    shared_ptr<KeyFrame> mpKFinitial;
    shared_ptr<KeyFrame> mpKFlowerID;
    shared_ptr<KeyFrame> mpNewestKF;

    std::vector<shared_ptr<MapPoint>> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;

};

} //namespace ORB_SLAM3

#endif // MAP_H
