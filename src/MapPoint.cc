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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint():
        mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0), mbOptimizedInLocalBA(false),
        mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mnIndexForSparsification(0),
        mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false), mnMapSparsificationId(0),
        mpReplaced(shared_ptr<MapPoint>()), mnUpdateChi2ForKF(0), mnOptimizedTimesInLBA(0), mbSparsified(false)
{
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, shared_ptr<KeyFrame> pRefKF, Map* pMap):
        mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0), mbOptimizedInLocalBA(false),
        mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mbSparsified(false),
        mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
        mpReplaced(shared_ptr<MapPoint>()), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mnIndexForSparsification(0), mnMapSparsificationId(0),
        mnOriginMapId(pMap->GetId()), mnOptimizedTimesInLBA(0)
{
    SetWorldPos(Pos);

    mNormalVector.setZero();

    mbTrackInViewR = false;
    mbTrackInView = false;
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, shared_ptr<KeyFrame> pRefKF, shared_ptr<KeyFrame> pHostKF, Map* pMap):
        mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0), mbSparsified(false), mbOptimizedInLocalBA(false),
        mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mnMapSparsificationId(0),
        mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
        mpReplaced(shared_ptr<MapPoint>()), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mnIndexForSparsification(0),
        mnOriginMapId(pMap->GetId()), mnUpdateChi2ForKF(0), mnOptimizedTimesInLBA(0)
{
    mNormalVector.setZero();
    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
        mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mbSparsified(false), mbOptimizedInLocalBA(false),
        mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mnIndexForSparsification(0), mnMapSparsificationId(0),
        mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(shared_ptr<KeyFrame>()), mnVisible(1),
        mnFound(1), mbBad(false), mpReplaced(shared_ptr<MapPoint>()), mpMap(pMap), mnOriginMapId(pMap->GetId()), mnUpdateChi2ForKF(0),
        mnOptimizedTimesInLBA(0)
{
    SetWorldPos(Pos);
    Eigen::Vector3f Ow;
    if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        Eigen::Matrix3f Rwl = pFrame->GetRwc();
        Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->GetOw();

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / mNormalVector.norm();

    Eigen::Vector3f PC = mWorldPos - Ow;
    const float dist = PC.norm();
    const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                              : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame -> mvKeysRight[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos() {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal() {
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector;
}


shared_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(shared_ptr<KeyFrame> pKF, int idx)
{
    float uRight = pKF->GetuRight(idx);
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int,int> indexes;

    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }

    if(pKF -> GetNLeft() != -1 && idx >= pKF -> GetNLeft()){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }

    mObservations[pKF]=indexes;

    if(!pKF->mpCamera2 && uRight>=0)
        nObs+=2;
    else
        nObs++;
}
    void MapPoint::UpdateObservation(shared_ptr<KeyFrame> pKF, int idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        tuple<int,int> indexes;
        if(mObservations.count(pKF)){
            indexes = mObservations[pKF];
        }
        else{
            indexes = tuple<int,int>(-1,-1);
        }
        if(pKF -> GetNLeft() != -1 && idx >= pKF -> GetNLeft()){
            get<1>(indexes) = idx;
        }
        else{
            get<0>(indexes) = idx;
        }
        mObservations[pKF]=indexes;
    }

void MapPoint::EraseObservation(shared_ptr<KeyFrame> pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
            float uRight = pKF->GetuRight(leftIndex);
            if(leftIndex != -1){
                if(!pKF->mpCamera2 && uRight>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(pKF);

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
            else if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

        }
    }

    if(bBad)
        SetBadFlag();
}


std::map<shared_ptr<KeyFrame>, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    if(isBad())
        return;
    map<shared_ptr<KeyFrame>, tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
        mpRefKF.reset();
        mpReplaced.reset();
    }
    for(map<shared_ptr<KeyFrame>, tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        shared_ptr<KeyFrame> pKF = mit->first;
        int leftIndex = get<0>(mit -> second), rightIndex = get<1>(mit -> second);
        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    shared_ptr<MapPoint> pThis = shared_from_this();
    mpMap->EraseMapPoint(pThis);
}

shared_ptr<MapPoint> MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(shared_ptr<MapPoint> pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<shared_ptr<KeyFrame>,tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<shared_ptr<KeyFrame>,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        shared_ptr<KeyFrame> pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(!pMP->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    shared_ptr<MapPoint> pThis = shared_from_this();
    mpMap->EraseMapPoint(pThis);
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mpRefKF.reset();
        mpReplaced.reset();
    }
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<shared_ptr<KeyFrame>,tuple<int,int>> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<shared_ptr<KeyFrame>,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        shared_ptr<KeyFrame> pKF = mit->first;

        if(!pKF->isBad()){
            tuple<int,int> indexes = mit -> second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                cv::Mat des = pKF->GetDescriptor(leftIndex);
                if(!des.empty())
                    vDescriptors.push_back(des);
            }
            if(rightIndex != -1){
                cv::Mat des = pKF->GetDescriptor(rightIndex);
                if(!des.empty())
                    vDescriptors.push_back(des);
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx];
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor;
}

tuple<int,int> MapPoint::GetIndexInKeyFrame(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return tuple<int,int>(-1,-1);
}

bool MapPoint::IsInKeyFrame(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<shared_ptr<KeyFrame>,tuple<int,int>> observations;
    shared_ptr<KeyFrame> pRefKF;
    Eigen::Vector3f Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos;
    }

    if(observations.empty())
        return;

    Eigen::Vector3f normal;
    normal.setZero();
    int n=0;
    for(map<shared_ptr<KeyFrame>,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        shared_ptr<KeyFrame> pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(leftIndex != -1){
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
        if(rightIndex != -1){
            Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
    const float dist = PC.norm();

    tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> GetNLeft() == -1){
        level = pRefKF->GetKeyUn(leftIndex).octave;
    }
    else if(leftIndex != -1){
        level = pRefKF ->GetKey(leftIndex).octave;
    }
    else{
        level = pRefKF ->GetKeyRight(rightIndex - pRefKF -> GetNLeft()).octave;
    }

    //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, shared_ptr<KeyFrame> pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

} //namespace ORB_SLAM
