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

#include "KeyFrame.h"
#include "Converter.h"
#include "ImuTypes.h"
#include<mutex>

namespace ORB_SLAM3
{

long unsigned int KeyFrame::nNextId=0;
int KeyFrame::mnNonLocalKF=0;

KeyFrame::KeyFrame():
        mnFrameId(0), mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0), mnCountInLocal(0),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0), mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), mPrevKF(shared_ptr<KeyFrame>()), mNextKF(shared_ptr<KeyFrame>()), mbFirstConnection(true), mpParent(shared_ptr<KeyFrame>()), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mnMergeCorrectedForKF(0),
        NLeft(0), NRight(0), mnNumberOfOpt(0), mbHasVelocity(false), mnMapSaprsificationId(0), mbSparsified(false), mbNonLocalKF(false)
{

}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, Tracking* pTracking):
        bImu(pMap->isImuInitialized()), mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy), mnCountInLocal(0),
        mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
        mvuRight(F.mvuRight), mvDepth(F.mvDepth), mpTracking(pTracking), mDescriptors(F.mDescriptors),
        mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
        mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
        mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX), mbNonLocalKF(false),
        mnMaxY(F.mnMaxY), mK_(F.mK_), mPrevKF(shared_ptr<KeyFrame>()), mNextKF(shared_ptr<KeyFrame>()), mpImuPreintegrated(F.mpImuPreintegrated),
        mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
        mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(shared_ptr<KeyFrame>()), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
        mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
        mpCamera(F.mpCamera), mpCamera2(F.mpCamera2), mbSparsified(false),
        mvLeftToRightMatch(F.mvLeftToRightMatch), mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
        mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0), mbHasVelocity(false), mnMapSaprsificationId(0)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }



    if(!F.HasVelocity()) {
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    mImuBias = F.mImuBias;
    SetPose(F.GetPose());

    mnOriginMapId = pMap->GetId();
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        unique_lock<mutex> lock(mMutexFeatures);
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexPose);

    mTcw = Tcw;
    mRcw = mTcw.rotationMatrix();
    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }
}

void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw)
{
    unique_lock<mutex> lock(mMutexPose);
    mVw = Vw;
    mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter(){
    unique_lock<mutex> lock(mMutexPose);
    return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

Eigen::Matrix3f KeyFrame::GetRotation(){
    unique_lock<mutex> lock(mMutexPose);
    return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return mVw;
}

bool KeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return mbHasVelocity;
}

void KeyFrame::AddConnection(shared_ptr<KeyFrame> pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,shared_ptr<KeyFrame>> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<shared_ptr<KeyFrame>,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<shared_ptr<KeyFrame>> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames.clear();
    mvpOrderedConnectedKeyFrames = vector<shared_ptr<KeyFrame>>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<shared_ptr<KeyFrame>> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<shared_ptr<KeyFrame>> s;
    for(map<shared_ptr<KeyFrame>,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<shared_ptr<KeyFrame>> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<shared_ptr<KeyFrame>> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<shared_ptr<KeyFrame>>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<shared_ptr<KeyFrame>> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return vector<shared_ptr<KeyFrame>>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<shared_ptr<KeyFrame>>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<shared_ptr<KeyFrame>>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int nObservedMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i] || mvpMapPoints[i]->isBad())
            continue;
        nObservedMPs++;
    }
    return nObservedMPs;
}

void KeyFrame::AddMapPoint(shared_ptr<MapPoint> pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx].reset();
}

void KeyFrame::EraseBadDescriptor() {
    shared_ptr<KeyFrame> pThis = shared_from_this();
//    mpKeyFrameDB->erase(pThis);
    {
        unique_lock<mutex> lock(mMutexFeatures);
        int nGoodMPS = 0;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (mvpMapPoints[i])
                nGoodMPS++;
        }
        cv::Mat DescriptorsNew;
        vector<cv::Mat> vCurrentDesc;
        vector<cv::KeyPoint> vKeysUnNew;
        vector<float> vuRightNew;
        vector<float> vDepthNew;
        vector<shared_ptr<MapPoint>> vpMapPointsNew;
        vCurrentDesc.reserve(nGoodMPS);
        vKeysUnNew.reserve(nGoodMPS);
        vuRightNew.reserve(nGoodMPS);
        vDepthNew.reserve(nGoodMPS);
        vpMapPointsNew.reserve(nGoodMPS);
        nGoodMPS = 0;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (mvpMapPoints[i]) {
                DescriptorsNew.push_back(mDescriptors.row(i));
                vCurrentDesc.push_back(mDescriptors.row(i));
                vKeysUnNew.push_back(mvKeysUn[i]);
                vuRightNew.push_back(mvuRight[i]);
                vDepthNew.push_back(mvDepth[i]);
                vpMapPointsNew.push_back(mvpMapPoints[i]);
                mvpMapPoints[i]->UpdateObservation(pThis, nGoodMPS);
                nGoodMPS++;
            }
        }
        mDescriptors.release();
        mDescriptors = DescriptorsNew;
        mvKeysUn.swap(vKeysUnNew);
        mvuRight.swap(vuRightNew);
        mvpMapPoints.swap(vpMapPointsNew);
        mvDepth.swap(vDepthNew);
        N = mvpMapPoints.size();
        mBowVec.clear();
        mFeatVec.clear();
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);

        std::vector<std::vector<std::vector<size_t> > >().swap(mGrid);
        std::vector<cv::KeyPoint>().swap(mvKeys);
        std::vector<cv::KeyPoint>().swap(mvKeysRight);
        mbSparsified = true;
    }
}

cv::Mat KeyFrame::GetDescriptor(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(idx >= mDescriptors.rows){
        cout << "This should not happen!!!" << endl;
        return cv::Mat();
    }
    return mDescriptors.row(idx);
}

void KeyFrame::EraseMapPointMatch(shared_ptr<MapPoint> pMP)
{
    shared_ptr<KeyFrame> pThis = shared_from_this();
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(pThis);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    unique_lock<mutex> lock(mMutexFeatures);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex].reset();
    if(rightIndex != -1)
        mvpMapPoints[rightIndex].reset();
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, shared_ptr<MapPoint> pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

set<shared_ptr<MapPoint>> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<shared_ptr<MapPoint>> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        shared_ptr<MapPoint> pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        shared_ptr<MapPoint> pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<shared_ptr<MapPoint>> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

shared_ptr<MapPoint> KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent)
{
    map<shared_ptr<KeyFrame>,int> KFcounter;

    vector<shared_ptr<MapPoint>> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(auto vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        shared_ptr<MapPoint> pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<shared_ptr<KeyFrame>,tuple<int,int>> observations = pMP->GetObservations();

        for(map<shared_ptr<KeyFrame>,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    if(mbSparsified){
        unique_lock<mutex> lockCon(mMutexConnections);
        for(auto mit=mConnectedKeyFrameWeights.begin(),mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++){
            KFcounter[mit->first] = mit->second;
        }
    }

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    shared_ptr<KeyFrame> pKFmax=shared_ptr<KeyFrame>();
    int th_1 = 15;
    int th_2 = 7;

    vector<pair<int,shared_ptr<KeyFrame>> > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    shared_ptr<KeyFrame> pThis = shared_from_this();
    for(map<shared_ptr<KeyFrame>,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend;)
    {
//        if(mbSparsified || mit->first->mbSparsified)
//            KFcounter[mit->first] = KFcounter[mit->first] + 90;
        int th = th_1;
        if(mbSparsified || mit->first->mbSparsified)
            th = th_2;
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(pThis,mit->second);
            mit++;
        }else{
            mit = KFcounter.erase(mit);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(pThis,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<shared_ptr<KeyFrame>> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<shared_ptr<KeyFrame>>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());


        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(pThis);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    shared_ptr<KeyFrame> pThis = shared_from_this();
    if(pKF == pThis)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    mpParent = pKF;
    pKF->AddChild(pThis);
}

set<shared_ptr<KeyFrame>> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

shared_ptr<KeyFrame> KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<shared_ptr<KeyFrame>> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

set<shared_ptr<KeyFrame>> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        unique_lock<mutex> lock(mMutexConnections);
        if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }
    shared_ptr<KeyFrame> pThis = shared_from_this();
    for(map<shared_ptr<KeyFrame>,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(pThis);
    }
    for (size_t i = 0; i < mvpMapPoints.size(); i++) {
        if (mvpMapPoints[i]) {
            mvpMapPoints[i]->EraseObservation(pThis);
            mvpMapPoints[i].reset();
        }
    }
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);
//        mvpMapPoints.clear();
        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<shared_ptr<KeyFrame>> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            shared_ptr<KeyFrame> pC;
            shared_ptr<KeyFrame> pP;

            for(set<shared_ptr<KeyFrame>>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                shared_ptr<KeyFrame> pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<shared_ptr<KeyFrame>> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<shared_ptr<KeyFrame>>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<shared_ptr<KeyFrame>>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(pThis);
            mTcp = mTcw * mpParent->GetPoseInverse();
        }
        mbBad = true;
    }

    mpTracking->UpdateSavedKeyFrame(pThis);
    mpMap->EraseKeyFrame(pThis);
    mpKeyFrameDB->erase(pThis);
    mPrevKF.reset();
    mNextKF.reset();
    mpParent.reset();
    mspChildrens.clear();
    mspLoopEdges.clear();
    mspMergeEdges.clear();
    mvpLoopCandKFs.clear();
    mvpMergeCandKFs.clear();

    mGrid.clear();
    mGridRight.clear();
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(shared_ptr<KeyFrame> pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight)
{
    vector<size_t> vIndices;
    unique_lock<mutex> lock(mMutexFeatures);
    if(mGrid.empty())
        return vIndices;


    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
{
    unique_lock<mutex> lock2(mMutexFeatures);
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        x3D = mRwc * x3Dc + mTwc.translation();
        return true;
    }
    else
        return false;
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    if(N==0)
        return -1.0;

    vector<shared_ptr<MapPoint>> vpMapPoints;
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        tcw = mTcw.translation();
        Rcw = mRcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
    float zcw = tcw(2);
    for(int i=0; i<N; i++) {
        if(vpMapPoints[i])
        {
            shared_ptr<MapPoint> pMP = vpMapPoints[i];
            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}


Sophus::SE3f KeyFrame::GetRelativePoseTrl()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTrl;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);

    return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTwc * mTlr).translation();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

bool KeyFrame::UpdateCountInLocalMapping(bool bLocal) {
    unique_lock<mutex> lock(mMutexFeatures);
    if (bLocal) {
        mnCountInLocal = 0;
        mbNonLocalKF = false;
        return false;
    } else {
        mnCountInLocal++;
        if (mnCountInLocal < mnNonLocalKF) {
            mbNonLocalKF = false;
            return false;
        }
        else {
            mbNonLocalKF = true;
            return true;
        }
    }
}

bool KeyFrame::UpdateCountInTracking(bool bLocal) {
    unique_lock<mutex> lock(mMutexFeatures);
    if (bLocal) {
        mnCountInLocal = 0;
        mbNonLocalKF = false;
        return false;
    } else {
        mnCountInLocal++;
        if (mnCountInLocal < mnNonLocalKF) {
            mbNonLocalKF = false;
            return false;
        }
        else {
            mbNonLocalKF = true;
            return true;
        }
    }
}

bool KeyFrame::isNonLocal() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mbNonLocalKF;
}

std::vector<cv::KeyPoint> KeyFrame::GetAllKeyUn(){
    unique_lock<mutex> lock(mMutexFeatures);
    return mvKeysUn;
}

DBoW2::BowVector KeyFrame::GetBowVector(){
    unique_lock<mutex> lock(mMutexFeatures);
    return mBowVec;
}

DBoW2::FeatureVector KeyFrame::GetFeatureVector(){
    unique_lock<mutex> lock(mMutexFeatures);
    return mFeatVec;
}

} //namespace ORB_SLAM
