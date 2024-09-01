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


#include "Map.h"

#include<mutex>

namespace ORB_SLAM3 {

long unsigned int Map::nNextId = 0;

Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0),
             mpFirstRegionKF(shared_ptr<KeyFrame>()),
             mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0),
             mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false) {
    mnId = nNextId++;
    mThumbnail = static_cast<GLubyte *>(NULL);
}

Map::Map(int initKFid) : mnInitKFid(initKFid), mnMaxKFid(initKFid), mnBigChangeIdx(0),
                         mIsInUse(false),
                         mHasTumbnail(false), mbBad(false), mbImuInitialized(false),
                         mpFirstRegionKF(shared_ptr<KeyFrame>()),
                         mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false),
                         mbIMU_BA1(false), mbIMU_BA2(false) {
    mnId = nNextId++;
    mThumbnail = static_cast<GLubyte *>(NULL);
}

Map::~Map() {
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if (mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte *>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(shared_ptr<KeyFrame> pKF) {
    unique_lock<mutex> lock(mMutexMap);
    if (mspKeyFrames.empty()) {
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if (pKF->mnId > mnMaxKFid) {
        mnMaxKFid = pKF->mnId;
    }
    if (pKF->mnId < mpKFlowerID->mnId) {
        mpKFlowerID = pKF;
    }
    mpNewestKF = pKF;
}

void Map::AddMapPoint(shared_ptr<MapPoint> pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}


void Map::AddSparsifiedMapPoint(shared_ptr<MapPoint> pMP) {
//    unique_lock<mutex> lock(mMutexMap);
    mspSparsifiedMapPoints.insert(pMP);
}

void Map::AddSparsifiedKeyFrame(shared_ptr<KeyFrame> pKF) {
    unique_lock<mutex> lock(mMutexMap);
    mspSparsifiedKeyFrames.insert(pKF);
    vector<shared_ptr<MapPoint>> vMPs = pKF->GetMapPointMatches();
    for (shared_ptr<MapPoint> pMPi: vMPs) {
        AddSparsifiedMapPoint(pMPi);
    }
}

void Map::SetImuInitialized() {
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized() {
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(shared_ptr<MapPoint> pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    mspSparsifiedMapPoints.erase(pMP);
}

void Map::EraseKeyFrame(shared_ptr<KeyFrame> pKF) {
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    mspSparsifiedKeyFrames.erase(pKF);
    if (mspKeyFrames.size() > 0) {
        if (pKF->mnId == mpKFlowerID->mnId) {
            vector<shared_ptr<KeyFrame> > vpKFs = vector<shared_ptr<KeyFrame> >(mspKeyFrames.begin(), mspKeyFrames.end());
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    } else {
        mpKFlowerID = 0;
    }
}

void Map::SetReferenceMapPoints(const vector<shared_ptr<MapPoint>> &vpMPs) {
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<shared_ptr<KeyFrame> > Map::GetAllKeyFrames() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<shared_ptr<KeyFrame> >(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<shared_ptr<KeyFrame> > Map::GetAllSparsifiedKeyFrames() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<shared_ptr<KeyFrame> >(mspSparsifiedKeyFrames.begin(), mspSparsifiedKeyFrames.end());
}

vector<shared_ptr<MapPoint>> Map::GetAllMapPoints() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<shared_ptr<MapPoint>>(mspMapPoints.begin(), mspMapPoints.end());
}

vector<shared_ptr<MapPoint>> Map::GetAllSparsifiedMapPoints() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<shared_ptr<MapPoint>>(mspSparsifiedMapPoints.begin(), mspSparsifiedMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::SparsifiedMapPointsInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspSparsifiedMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<shared_ptr<MapPoint>> Map::GetReferenceMapPoints() {
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId() {
    return mnId;
}

long unsigned int Map::GetInitKFid() {
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif) {
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid() {
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

shared_ptr<KeyFrame> Map::GetOriginKF() {
    return mpKFinitial;
}

shared_ptr<KeyFrame> Map::GetNewestKF() {
    unique_lock<mutex> lock(mMutexMap);
    return mpNewestKF;
}

void Map::SetCurrentMap() {
    mIsInUse = true;
}

void Map::SetStoredMap() {
    mIsInUse = false;
}

void Map::clear() {
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    for (set<shared_ptr<KeyFrame> >::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++) {
        shared_ptr<KeyFrame> pKF = *sit;
        pKF->UpdateMap(static_cast<Map *>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mspSparsifiedMapPoints.clear();
    mspSparsifiedKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse() {
    return mIsInUse;
}

void Map::SetBad() {
    mbBad = true;
}

bool Map::IsBad() {
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel) {
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for (set<shared_ptr<KeyFrame> >::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++) {
        shared_ptr<KeyFrame> pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if (!bScaledVel)
            pKF->SetVelocity(Ryw * Vw);
        else
            pKF->SetVelocity(Ryw * Vw * s);

    }
    for (set<shared_ptr<MapPoint>>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++) {
        shared_ptr<MapPoint> pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor() {
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial() {
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1() {
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2() {
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1() {
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2() {
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId) {
    mnId = nId;
}

unsigned int Map::GetLowerKFID() {
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex() {
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex() {
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange() {
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId) {
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

} //namespace ORB_SLAM3
