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


#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"
#include "MLPnPsolver.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM3
{

    LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bActiveLC):
            mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
            mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
            mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0), mnMergeNumCoincidences(0),
            mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0), mbActiveLC(bActiveLC)
    {
        mnCovisibilityConsistencyTh = 3;
        mpLastCurrentKF = static_cast<shared_ptr<KeyFrame> >(NULL);

#ifdef REGISTER_TIMES

        vdDataQuery_ms.clear();
    vdEstSim3_ms.clear();
    vdPRTotal_ms.clear();

    vdMergeMaps_ms.clear();
    vdWeldingBA_ms.clear();
    vdMergeOptEss_ms.clear();
    vdMergeTotal_ms.clear();
    vnMergeKFs.clear();
    vnMergeMPs.clear();
    nMerges = 0;

    vdLoopFusion_ms.clear();
    vdLoopOptEss_ms.clear();
    vdLoopTotal_ms.clear();
    vnLoopKFs.clear();
    nLoop = 0;

    vdGBA_ms.clear();
    vdUpdateMap_ms.clear();
    vdFGBATotal_ms.clear();
    vnGBAKFs.clear();
    vnGBAMPs.clear();
    nFGBA_exec = 0;
    nFGBA_abort = 0;

#endif

        mstrFolderSubTraj = "SubTrajectories/";
        mnNumCorrection = 0;
        mnCorrectionGBA = 0;
    }

    void LoopClosing::SetTracker(Tracking *pTracker)
    {
        mpTracker=pTracker;
    }

    void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper=pLocalMapper;
    }

    void LoopClosing::SetMapSparsification(MapSparsification* pMapSparsification)
    {
        mpMapSparsification=pMapSparsification;
    }

    void LoopClosing::Run()
    {
        mbFinished =false;
        int nLoops = 0;
        while(1)
        {

            //NEW LOOP AND MERGE DETECTION ALGORITHM
            //----------------------------
            DeleteOutdatedInfo();

            if(CheckNewKeyFrames())
            {
                if(mpLastCurrentKF)
                {
                    mpLastCurrentKF->mvpLoopCandKFs.clear();
                    mpLastCurrentKF->mvpMergeCandKFs.clear();
                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartPR = std::chrono::steady_clock::now();
#endif

                bool bFindedRegion = NewDetectCommonRegions();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndPR = std::chrono::steady_clock::now();

            double timePRTotal = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPR - time_StartPR).count();
            vdPRTotal_ms.push_back(timePRTotal);
#endif
                if(bFindedRegion)
                {
                    if(mbMergeDetected)
                    {
                        if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                            (!mpCurrentKF->GetMap()->isImuInitialized()))
                        {
                            cout << "IMU is not initilized, merge is aborted" << endl;
                        }
                        else
                        {
                            Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
                            g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
                            Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
                            g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
                            g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                            g2o::Sim3 gSw1m = mg2oMergeSlw;

                            mSold_new = (gSw2c * gScw1);

                            if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                            {
                                cout << "Merge check transformation with IMU" << endl;
                                if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
                                    mpMergeLastCurrentKF->SetErase();
                                    mpMergeMatchedKF->SetErase();
                                    mnMergeNumCoincidences = 0;
                                    mvpMergeMatchedMPs.clear();
                                    mvpMergeMPs.clear();
                                    mnMergeNumNotFound = 0;
                                    mbMergeDetected = false;
                                    Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                    continue;
                                }
                                // If inertial, force only yaw
                                if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                                    mpCurrentKF->GetMap()->GetIniertialBA1())
                                {
                                    Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                    phi(0)=0;
                                    phi(1)=0;
                                    mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
                                }
                            }

                            mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                            mg2oMergeScw = mg2oMergeSlw;

                            //mpTracker->SetStepByStep(true);

                            Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();

                        nMerges += 1;
#endif
                            // TODO UNCOMMENT
                            if (mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
                                MergeLocal2();
                            else
                                MergeLocal();

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();

                        double timeMergeTotal = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMerge - time_StartMerge).count();
                        vdMergeTotal_ms.push_back(timeMergeTotal);
#endif

                            Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                        }

                        vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                        vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                        vnPR_TypeRecogn.push_back(1);

                        // Reset all variables
                        mpMergeLastCurrentKF->SetErase();
                        mpMergeMatchedKF->SetErase();
                        mnMergeNumCoincidences = 0;
                        mvpMergeMatchedMPs.clear();
                        mvpMergeMPs.clear();
                        mnMergeNumNotFound = 0;
                        mbMergeDetected = false;

                        if(mbLoopDetected)
                        {
                            // Reset Loop variables
                            mpLoopLastCurrentKF->SetErase();
                            mpLoopMatchedKF->SetErase();
                            mnLoopNumCoincidences = 0;
                            mvpLoopMatchedMPs.clear();
                            mvpLoopMPs.clear();
                            mnLoopNumNotFound = 0;
                            mbLoopDetected = false;
                        }

                    }

                    if(mbLoopDetected)
                    {
                        bool bGoodLoop = true;
                        vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                        vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                        vnPR_TypeRecogn.push_back(0);

//                        Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);
                        mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
                        if(mpCurrentKF->GetMap()->IsInertial())
                        {
                            Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
                            g2o::Sim3 g2oTwc(Twc.unit_quaternion(),Twc.translation(),1.0);
                            g2o::Sim3 g2oSww_new = g2oTwc*mg2oLoopScw;

                            Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                            cout << "phi = " << phi.transpose() << endl;
                            if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)
                            {
                                if(mpCurrentKF->GetMap()->IsInertial())
                                {
                                    // If inertial, force only yaw
                                    if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                                        mpCurrentKF->GetMap()->GetIniertialBA2())
                                    {
                                        phi(0)=0;
                                        phi(1)=0;
                                        g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
                                        mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
                                    }
                                }

                            }
                            else
                            {
                                cout << "BAD LOOP!!!" << endl;
                                bGoodLoop = false;
                            }

                        }
                        if (bGoodLoop) {
                            mvpLoopMapPoints = mvpLoopMPs;
                            CorrectLoop();
                            mnNumCorrection += 1;
                        }

                        // Reset all variables
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mvpLoopMapPoints.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }

                }
                mpLastCurrentKF = mpCurrentKF;
            }

            ResetIfRequested();

            if(CheckFinish()){
                break;
            }

            usleep(5000);
        }

        SetFinish();
    }

    void LoopClosing::InsertKeyFrame(shared_ptr<KeyFrame> pKF)
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        if(pKF->mnId!=0)
            mlpLoopKeyFrameQueue.push_back(pKF);
    }

    void LoopClosing::InsertSparsifiedKeyFrame(shared_ptr<KeyFrame> pKF)
    {
        unique_lock<mutex> lock(mMutexLoopQueue2);
        mlpSparsifiedKeyFrameQueue.push_back(pKF);
    }

    bool LoopClosing::CheckNewKeyFrames()
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        return(!mlpLoopKeyFrameQueue.empty());
    }

    void LoopClosing::DeleteOutdatedInfo() {
        unique_lock<mutex> lock(mMutexLoopQueue2);
        Map *pCurrentMap = mpAtlas->GetCurrentMap();
        while(!mlpSparsifiedKeyFrameQueue.empty()) {
            shared_ptr<KeyFrame> pKF = mlpSparsifiedKeyFrameQueue.front();
            mlpSparsifiedKeyFrameQueue.pop_front();
            pKF->EraseBadDescriptor();
            mpKeyFrameDB->add(pKF);
            pCurrentMap->AddSparsifiedKeyFrame(pKF);
        }
    }

    bool LoopClosing::NewDetectCommonRegions()
    {
        // To deactivate placerecognition. No loopclosing nor merging will be performed
        if(!mbActiveLC)
            return false;

        {
            unique_lock<mutex> lock(mMutexLoopQueue);
            mpCurrentKF = mlpLoopKeyFrameQueue.front();
            mlpLoopKeyFrameQueue.pop_front();
            // Avoid that a keyframe can be erased while it is being process by this thread
            mpCurrentKF->SetNotErase();
            mpCurrentKF->mbCurrentPlaceRecognition = true;

            mpLastMap = mpCurrentKF->GetMap();
        }

        if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())
        {
            mpCurrentKF->SetErase();
            return false;
        }

        if(mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
        {
            // cout << "LoopClousure: Stereo KF inserted without check: " << mpCurrentKF->mnId << endl;
            mpCurrentKF->SetErase();
            return false;
        }

        if(mpLastMap->GetAllKeyFrames().size() < 12)
        {
            // cout << "LoopClousure: Stereo KF inserted without check, map is small: " << mpCurrentKF->mnId << endl;
            mpCurrentKF->SetErase();
            return false;
        }

        //cout << "LoopClousure: Checking KF: " << mpCurrentKF->mnId << endl;

        //Check the last candidates with geometric validation
        // Loop candidates
        bool bLoopDetectedInKF = false;
        bool bCheckSpatial = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartEstSim3_1 = std::chrono::steady_clock::now();
#endif
        if(mnLoopNumCoincidences > 0)
        {
            bCheckSpatial = true;
            // Find from the last KF candidates
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
            g2o::Sim3 gScl(mTcl.unit_quaternion(),mTcl.translation(),1.0);
            g2o::Sim3 gScw = gScl * mg2oLoopSlw;
            int numProjMatches = 0;
            vector<shared_ptr<MapPoint> > vpMatchedMPs;
            bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);
            if(bCommonRegion)
            {

                bLoopDetectedInKF = true;

                mnLoopNumCoincidences++;
                mpLoopLastCurrentKF->SetErase();
                mpLoopLastCurrentKF = mpCurrentKF;
                mg2oLoopSlw = gScw;
                mvpLoopMatchedMPs = vpMatchedMPs;


                mbLoopDetected = mnLoopNumCoincidences >= 3;
                mnLoopNumNotFound = 0;

                if(!mbLoopDetected)
                {
                    cout << "PR: Loop detected with Reffine Sim3" << endl;
                }
            }
            else
            {
                bLoopDetectedInKF = false;

                mnLoopNumNotFound++;
                if(mnLoopNumNotFound >= 2)
                {
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                }

            }
        }

        //Merge candidates
        bool bMergeDetectedInKF = false;
        if(mnMergeNumCoincidences > 0)
        {
            // Find from the last KF candidates
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
            g2o::Sim3 gScw = gScl * mg2oMergeSlw;
            int numProjMatches = 0;
            vector<shared_ptr<MapPoint> > vpMatchedMPs;
            bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
            if(bCommonRegion)
            {
                bMergeDetectedInKF = true;

                mnMergeNumCoincidences++;
                mpMergeLastCurrentKF->SetErase();
                mpMergeLastCurrentKF = mpCurrentKF;
                mg2oMergeSlw = gScw;
                mvpMergeMatchedMPs = vpMatchedMPs;

                mbMergeDetected = mnMergeNumCoincidences >= 3;
            }
            else
            {
                mbMergeDetected = false;
                bMergeDetectedInKF = false;

                mnMergeNumNotFound++;
                if(mnMergeNumNotFound >= 2)
                {
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                }


            }
        }
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_1 = std::chrono::steady_clock::now();

        double timeEstSim3 = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndEstSim3_1 - time_StartEstSim3_1).count();
#endif

        if(mbMergeDetected || mbLoopDetected)
        {
#ifdef REGISTER_TIMES
            vdEstSim3_ms.push_back(timeEstSim3);
#endif
//            mpKeyFrameDB->add(mpCurrentKF);
            return true;
        }

        //TODO: This is only necessary if we use a minimun score for pick the best candidates
        const vector<shared_ptr<KeyFrame> > vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

        // Extract candidates from the bag of words
        vector<shared_ptr<KeyFrame> > vpMergeBowCand, vpLoopBowCand;
        if(!bMergeDetectedInKF || !bLoopDetectedInKF)
        {
            // Search in BoW
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartQuery = std::chrono::steady_clock::now();
#endif

            mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 6);
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndQuery = std::chrono::steady_clock::now();

        double timeDataQuery = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndQuery - time_StartQuery).count();
        vdDataQuery_ms.push_back(timeDataQuery);
#endif
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartEstSim3_2 = std::chrono::steady_clock::now();
#endif
        // Check the BoW candidates if the geometric candidate list is empty
        //Loop candidates
        if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
        {
            mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
        }
        // Merge candidates
        if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
        {
            mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_2 = std::chrono::steady_clock::now();

        timeEstSim3 += std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndEstSim3_2 - time_StartEstSim3_2).count();
        vdEstSim3_ms.push_back(timeEstSim3);
#endif

//        mpKeyFrameDB->add(mpCurrentKF);

        if(mbMergeDetected || mbLoopDetected)
        {
            return true;
        }

        mpCurrentKF->SetErase();
        mpCurrentKF->mbCurrentPlaceRecognition = false;

        return false;
    }

    bool LoopClosing::DetectAndReffineSim3FromLastKF(shared_ptr<KeyFrame>  pCurrentKF, shared_ptr<KeyFrame>  pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                     std::vector<shared_ptr<MapPoint> > &vpMPs, std::vector<shared_ptr<MapPoint> > &vpMatchedMPs)
    {
        set<shared_ptr<MapPoint> > spAlreadyMatchedMPs;
        nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

        int nProjMatches = 30;
        int nProjOptMatches = 50;
        int nProjMatchesRep = 100;

        if(nNumProjMatches >= nProjMatches)
        {
            //Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumProjMatches) + " initial matches ", Verbose::VERBOSITY_DEBUG);
            Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
            g2o::Sim3 gSwm(mTwm.unit_quaternion(),mTwm.translation(),1.0);
            g2o::Sim3 gScm = gScw * gSwm;
            Eigen::Matrix<double, 7, 7> mHessian7x7;

            bool bFixedScale = mbFixScale;       // TODO CHECK; Solo para el monocular inertial
            if(mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale=false;
            int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

            //Verbose::PrintMess("Sim3 reffine: There are " + to_string(numOptMatches) + " matches after of the optimization ", Verbose::VERBOSITY_DEBUG);

            if(numOptMatches > nProjOptMatches)
            {
                g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(),1.0);

                vector<shared_ptr<MapPoint> > vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<shared_ptr<MapPoint> >(NULL));

                nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
                if(nNumProjMatches >= nProjMatchesRep)
                {
                    gScw = gScw_estimation;
                    return true;
                }
            }
        }
        return false;
    }

    bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<shared_ptr<KeyFrame> > &vpBowCand, shared_ptr<KeyFrame>  &pMatchedKF2, shared_ptr<KeyFrame>  &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                                 int &nNumCoincidences, std::vector<shared_ptr<MapPoint> > &vpMPs, std::vector<shared_ptr<MapPoint> > &vpMatchedMPs)
    {
        int nBoWMatches = 20;
        int nBoWInliers = 12;
        int nSim3Inliers = 20;
        int nProjMatches = 25;
        int nProjOptMatches = 30;
        set<shared_ptr<KeyFrame> > spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

        int nNumCovisibles = 10;

        ORBmatcher matcherBoW(0.9, true);
        ORBmatcher matcher(0.75, true);

        // Varibles to select the best numbe
        shared_ptr<KeyFrame>  pBestMatchedKF;
        int nBestMatchesReproj = 0;
        int nBestNumCoindicendes = 0;
        g2o::Sim3 g2oBestScw;
        std::vector<shared_ptr<MapPoint> > vpBestMapPoints;
        std::vector<shared_ptr<MapPoint> > vpBestMatchedMapPoints;

        int numCandidates = vpBowCand.size();
        vector<int> vnStage(numCandidates, 0);
        vector<int> vnMatchesStage(numCandidates, 0);

        int index = 0;
        //Verbose::PrintMess("BoW candidates: There are " + to_string(vpBowCand.size()) + " possible candidates ", Verbose::VERBOSITY_DEBUG);
        for(shared_ptr<KeyFrame>  pKFi : vpBowCand)
        {
            if(!pKFi || pKFi->isBad())
                continue;

            // std::cout << "KF candidate: " << pKFi->mnId << std::endl;
            // Current KF against KF with covisibles version
            std::vector<shared_ptr<KeyFrame> > vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
            std::vector<shared_ptr<KeyFrame> > vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
            if(vpCovKFi.empty())
            {
                std::cout << "Covisible list empty" << std::endl;
                vpCovKFi.push_back(pKFi);
            }
            else
            {
                vpCovKFi.push_back(vpCovKFi[0]);
                vpCovKFi[0] = pKFi;
            }


            bool bAbortByNearKF = false;
            for(int j=0; j<vpCovKFi.size(); ++j)
            {
                if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
                {
                    bAbortByNearKF = true;
                    break;
                }
            }
            if(bAbortByNearKF)
            {
                //std::cout << "Check BoW aborted because is close to the matched one " << std::endl;
                continue;
            }
            //std::cout << "Check BoW continue because is far to the matched one " << std::endl;

            vector<shared_ptr<KeyFrame>> vpMatchedCurrentKeyFrame;
            vector<shared_ptr<MapPoint>> vpMatchedCurrentMapPoint;
            vector<shared_ptr<KeyFrame>> vpMatchedLoopKeyFrame;
            vector<shared_ptr<MapPoint>> vpMatchedLoopMapPoint;
            int numBoWMatches = 0;
            for(int j=0; j<vpCovKFi.size(); ++j)
            {
                if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                    continue;

                int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vpMatchedCurrentKeyFrame, vpMatchedCurrentMapPoint,
                                                 vpMatchedLoopKeyFrame, vpMatchedLoopMapPoint, mpCurrentKF->mnId);
                numBoWMatches+=num;
            }
            for (shared_ptr<KeyFrame> pCurrentCovKFi : vpCurrentCovKFs) {
                if(!pCurrentCovKFi || pCurrentCovKFi->isBad())
                    continue;
                for(int j=0; j<vpCovKFi.size(); ++j)
                {
                    if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                        continue;
                    int num = matcherBoW.SearchByBoW(pCurrentCovKFi, vpCovKFi[j], vpMatchedCurrentKeyFrame, vpMatchedCurrentMapPoint,
                                                     vpMatchedLoopKeyFrame, vpMatchedLoopMapPoint, mpCurrentKF->mnId);
                    numBoWMatches+=num;
                }
            }


            if(numBoWMatches >= nBoWMatches) // TODO pick a good threshold
            {
                // Geometric validation
                bool bFixedScale = mbFixScale;
                if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                    bFixedScale=false;

                Sim3Solver solver = Sim3Solver(mpCurrentKF, pKFi, vpMatchedCurrentKeyFrame, vpMatchedCurrentMapPoint,
                                               vpMatchedLoopKeyFrame, vpMatchedLoopMapPoint, bFixedScale);
                solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

                bool bNoMore = false;
                vector<bool> vbInliers;
                int nInliers;
                bool bConverge = false;
                Eigen::Matrix4f mTcm;
                while(!bConverge && !bNoMore)
                {
                    mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
                    //Verbose::PrintMess("BoW guess: Solver achieve " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                }
                if(bConverge)
                {
                    // Match by reprojection
                    vpCovKFi.push_back(pKFi);
                    set<shared_ptr<MapPoint> > spMapPoints;
                    vector<shared_ptr<MapPoint> > vpLoopMapPoints;
                    vector<shared_ptr<MapPoint> > vpMatchedCurrentMP;
                    vector<shared_ptr<KeyFrame> > vpLoopKeyFrames;
                    vector<shared_ptr<KeyFrame> > vpMatchedCurrentKF;
                    int numProjMatches = 0;
                    for (int i = 0; i < vbInliers.size(); ++i) {
                        if(vbInliers[i]){
                            spMapPoints.insert(vpMatchedLoopMapPoint[i]);
                            vpLoopMapPoints.push_back(vpMatchedLoopMapPoint[i]);
                            vpMatchedCurrentMP.push_back(vpMatchedCurrentMapPoint[i]);

                            vpLoopKeyFrames.push_back(vpMatchedLoopKeyFrame[i]);
                            vpMatchedCurrentKF.push_back(vpMatchedCurrentKeyFrame[i]);
                            numProjMatches++;
                        }
                    }


                    for(shared_ptr<KeyFrame>  pCovKFi : vpCovKFi)
                    {
                        const std::vector<shared_ptr<MapPoint>> vMapPointsToMatch = pCovKFi->GetMapPointMatches();
                        for(shared_ptr<MapPoint>  pCovMPij : vMapPointsToMatch)
                        {
                            if(!pCovMPij || pCovMPij->isBad())
                                continue;

                            if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                            {
                                spMapPoints.insert(pCovMPij);
                                vpLoopMapPoints.push_back(pCovMPij);
                                vpLoopKeyFrames.push_back(pCovKFi);
                            }
                        }
                    }
                    spMapPoints.clear();
                    vpMatchedCurrentMP.resize(vpLoopMapPoints.size(), static_cast<shared_ptr<MapPoint> >(NULL));
                    vpMatchedCurrentKF.resize(vpLoopMapPoints.size(), static_cast<shared_ptr<KeyFrame> >(NULL));

                    g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),solver.GetEstimatedTranslation().cast<double>(), (double) solver.GetEstimatedScale());
                    g2o::Sim3 gSmw(pKFi->GetRotation().cast<double>(),pKFi->GetTranslation().cast<double>(),1.0);
                    g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                    Sophus::Sim3f mScw = Converter::toSophus(gScw);
                    numProjMatches += matcher.SearchByProjectionLoop(mpCurrentKF, mScw, vpLoopMapPoints, vpMatchedCurrentMP, vpMatchedCurrentKF, 8, 1.5);

                    Sophus::SE3f Tw_c = mpCurrentKF->GetPoseInverse();
                    for (shared_ptr<KeyFrame> pCurrentCovKFi : vpCurrentCovKFs) {
                        Sophus::SE3f Tciw_ = pCurrentCovKFi->GetPose();
                        Sophus::SE3f Tcic = Tciw_ * Tw_c;
                        g2o::Sim3 gScic(Tcic.rotationMatrix().cast<double>(), Tcic.translation().cast<double>(), 1.0);
                        g2o::Sim3 gSciw = gScic * gScw;
                        Sophus::Sim3f mSciw = Converter::toSophus(gSciw);
                        numProjMatches += matcher.SearchByProjectionLoop(pCurrentCovKFi, mScw, vpLoopMapPoints, vpMatchedCurrentMP, vpMatchedCurrentKF, 8, 1.5);
                    }

                    if(numProjMatches >= nProjMatches)
                    {
                        // Optimize Sim3 transformation with every matches
                        Eigen::Matrix<double, 7, 7> mHessian7x7;

                        bool bFixedScale = mbFixScale;
                        if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                            bFixedScale=false;

                        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedCurrentMP, vpLoopMapPoints, gScm, 10, mbFixScale, mHessian7x7, true);
                        if(numOptMatches >= nSim3Inliers)
                        {
                            g2o::Sim3 gSmw(pKFi->GetRotation().cast<double>(),pKFi->GetTranslation().cast<double>(),1.0);
                            g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                            Sophus::Sim3f mScw = Converter::toSophus(gScw);

                            vector<shared_ptr<MapPoint> > vpMatchedMP;
                            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<shared_ptr<MapPoint> >(NULL));
                            int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpLoopMapPoints, vpMatchedMP, 5, 1.0);
                            if(numProjOptMatches >= nProjOptMatches)
                            {
                                int nNumKFs = 0;
                                // Check the Sim3 transformation with the current KeyFrame covisibles

                                int j = 0;
                                while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                                {
                                    shared_ptr<KeyFrame>  pKFj = vpCurrentCovKFs[j];
                                    Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                                    g2o::Sim3 gSjc(mTjc.unit_quaternion(),mTjc.translation(),1.0);
                                    g2o::Sim3 gSjw = gSjc * gScw;
                                    int numProjMatches_j = 0;
                                    vector<shared_ptr<MapPoint> > vpMatchedMPs_j;
                                    bool bValid = DetectCommonRegionsFromLastKF(pKFj, pKFi, gSjw,numProjMatches_j, vpLoopMapPoints, vpMatchedMPs_j);

                                    if(bValid)
                                    {
                                        Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                        Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                        Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                        Eigen::Vector3f vector_dist = Tc_cj.translation();
                                        nNumKFs++;
                                    }
                                    j++;
                                }

                                if(nNumKFs < 3)
                                {
                                    vnStage[index] = 8;
                                    vnMatchesStage[index] = nNumKFs;
                                }

                                if(nBestMatchesReproj < numProjOptMatches)
                                {
                                    nBestMatchesReproj = numProjOptMatches;
                                    nBestNumCoindicendes = nNumKFs;
                                    pBestMatchedKF = pKFi;
                                    g2oBestScw = gScw;
                                    vpBestMapPoints = vpLoopMapPoints;
                                    vpBestMatchedMapPoints = vpMatchedMP;
                                }
                            }
                        }
                    }
                }
            }
            index++;
        }

        if(nBestMatchesReproj > 0)
        {
            pLastCurrentKF = mpCurrentKF;
            nNumCoincidences = nBestNumCoindicendes;
            pMatchedKF2 = pBestMatchedKF;
            pMatchedKF2->SetNotErase();
            g2oScw = g2oBestScw;
            vpMPs = vpBestMapPoints;
            vpMatchedMPs = vpBestMatchedMapPoints;

            return nNumCoincidences >= 3;
        }
        else
        {
            int maxStage = -1;
            int maxMatched;
            for(int i=0; i<vnStage.size(); ++i)
            {
                if(vnStage[i] > maxStage)
                {
                    maxStage = vnStage[i];
                    maxMatched = vnMatchesStage[i];
                }
            }
        }
        return false;
    }

    bool LoopClosing::DetectCommonRegionsFromLastKF(shared_ptr<KeyFrame>  pCurrentKF, shared_ptr<KeyFrame>  pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                    std::vector<shared_ptr<MapPoint> > &vpMPs, std::vector<shared_ptr<MapPoint> > &vpMatchedMPs)
    {
        set<shared_ptr<MapPoint> > spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
        nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

        int nProjMatches = 20;
        if(nNumProjMatches >= nProjMatches)
        {
            return true;
        }

        return false;
    }

    int LoopClosing::FindMatchesByProjection(shared_ptr<KeyFrame>  pCurrentKF, shared_ptr<KeyFrame>  pMatchedKFw, g2o::Sim3 &g2oScw,
                                             set<shared_ptr<MapPoint> > &spMatchedMPinOrigin, vector<shared_ptr<MapPoint> > &vpMapPoints,
                                             vector<shared_ptr<MapPoint> > &vpMatchedMapPoints)
    {
        int nNumCovisibles = 10;
        vector<shared_ptr<KeyFrame> > vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
        int nInitialCov = vpCovKFm.size();
        vpCovKFm.push_back(pMatchedKFw);
        set<shared_ptr<KeyFrame> > spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
        set<shared_ptr<KeyFrame> > spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
        if(nInitialCov < nNumCovisibles)
        {
            for(int i=0; i<nInitialCov; ++i)
            {
                vector<shared_ptr<KeyFrame> > vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
                int nInserted = 0;
                int j = 0;
                while(j < vpKFs.size() && nInserted < nNumCovisibles)
                {
                    if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
                    {
                        spCheckKFs.insert(vpKFs[j]);
                        ++nInserted;
                    }
                    ++j;
                }
                vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
            }
        }
        set<shared_ptr<MapPoint> > spMapPoints;
        vpMapPoints.clear();
        vpMatchedMapPoints.clear();
        for(shared_ptr<KeyFrame>  pKFi : vpCovKFm)
        {
            for(shared_ptr<MapPoint>  pMPij : pKFi->GetMapPointMatches())
            {
                if(!pMPij || pMPij->isBad())
                    continue;

                if(spMapPoints.find(pMPij) == spMapPoints.end())
                {
                    spMapPoints.insert(pMPij);
                    vpMapPoints.push_back(pMPij);
                }
            }
        }

        Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
        ORBmatcher matcher(0.9, true);

        vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<shared_ptr<MapPoint> >(NULL));
        int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

        return num_matches;
    }

    void LoopClosing::CorrectLoop()
    {
        // Send a stop signal to Local Mapping
        // Avoid new keyframes are inserted while correcting the loop
        mpLocalMapper->RequestStop();
        mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue
        mpMapSparsification->RequestStop();

        // If a Global Bundle Adjustment is running, abort it
        if(isRunningGBA())
        {
            cout << "Stoping Global Bundle Adjustment...";
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if(mpThreadGBA)
            {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            cout << "  Done!!" << endl;
        }

        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        while(!mpMapSparsification->isStopped())
        {
            usleep(1000);
        }

        // Ensure current keyframe is updated
        mpCurrentKF->UpdateConnections();

        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        std::vector<shared_ptr<KeyFrame>> vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurrentConnectedKFs.push_back(mpCurrentKF);

        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        CorrectedSim3[mpCurrentKF]=mg2oLoopScw;
        Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
        Sophus::SE3f Tcw = mpCurrentKF->GetPose();
        g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>(),1.0);
        NonCorrectedSim3[mpCurrentKF]=g2oScw;

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),mg2oLoopScw.translation() / mg2oLoopScw.scale());
        mpCurrentKF->SetPose(correctedTcw.cast<float>());

        Map* pLoopMap = mpCurrentKF->GetMap();

#ifdef REGISTER_TIMES
        /*shared_ptr<KeyFrame>  pKF = mpCurrentKF;
    int numKFinLoop = 0;
    while(pKF && pKF->mnId > mpLoopMatchedKF->mnId)
    {
        pKF = pKF->GetParent();
        numKFinLoop += 1;
    }
    vnLoopKFs.push_back(numKFinLoop);*/

    std::chrono::steady_clock::time_point time_StartFusion = std::chrono::steady_clock::now();
#endif

        {
            // Get Map Mutex
            unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

            const bool bImuInit = pLoopMap->isImuInitialized();

            for(vector<shared_ptr<KeyFrame> >::iterator vit=vpCurrentConnectedKFs.begin(), vend=vpCurrentConnectedKFs.end(); vit!=vend; vit++)
            {
                shared_ptr<KeyFrame>  pKFi = *vit;

                if(pKFi!=mpCurrentKF)
                {
                    Sophus::SE3f Tiw = pKFi->GetPose();
                    Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                    g2o::Sim3 g2oSic(Tic.unit_quaternion(),Tic.translation(),1.0);
                    g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oLoopScw;
                    //Pose corrected with the Sim3 of the loop closure
                    CorrectedSim3[pKFi]=g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                    pKFi->SetPose(correctedTiw.cast<float>());

                    //Pose without correction
                    g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),Tiw.translation().cast<double>(),1.0);
                    NonCorrectedSim3[pKFi]=g2oSiw;
                }
            }

            // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
            for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
            {
                shared_ptr<KeyFrame>  pKFi = mit->first;
                g2o::Sim3 g2oCorrectedSiw = mit->second;
                g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

                g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                pKFi->SetPose(correctedTiw.cast<float>());*/

                vector<shared_ptr<MapPoint> > vpMPsi = pKFi->GetMapPointMatches();
                for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
                {
                    shared_ptr<MapPoint>  pMPi = vpMPsi[iMP];
                    if(!pMPi)
                        continue;
                    if(pMPi->isBad())
                        continue;
                    if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                        continue;

                    // Project with non-corrected pose and project back with corrected pose
                    Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                    Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                    pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                    pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                    pMPi->mnCorrectedReference = pKFi->mnId;
                    pMPi->UpdateNormalAndDepth();
                }

                // Correct velocity according to orientation correction
                if(bImuInit)
                {
                    Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse()*g2oSiw.rotation()).cast<float>();
                    pKFi->SetVelocity(Rcor*pKFi->GetVelocity());
                }

                // Make sure connections are updated
                pKFi->UpdateConnections();
            }
            // TODO Check this index increasement
            mpAtlas->GetCurrentMap()->IncreaseChangeIndex();


            // Start Loop Fusion
            // Update matched map points and replace if duplicated
            for(size_t i=0; i<mvpLoopMatchedMPs.size(); i++)
            {
                if(mvpLoopMatchedMPs[i])
                {
                    shared_ptr<MapPoint>  pLoopMP = mvpLoopMatchedMPs[i];
                    shared_ptr<MapPoint>  pCurMP = mpCurrentKF->GetMapPoint(i);
                    if(pCurMP)
                        pCurMP->Replace(pLoopMP);
                    else
                    {
                        mpCurrentKF->AddMapPoint(pLoopMP,i);
                        pLoopMP->AddObservation(mpCurrentKF,i);
                        pLoopMP->ComputeDistinctiveDescriptors();
                    }
                }
            }
            //cout << "LC: end replacing duplicated" << endl;
        }

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        map<shared_ptr<KeyFrame> , set<shared_ptr<KeyFrame> > > LoopConnections;

        for(vector<shared_ptr<KeyFrame> >::iterator vit=vpCurrentConnectedKFs.begin(), vend=vpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            shared_ptr<KeyFrame>  pKFi = *vit;
            vector<shared_ptr<KeyFrame> > vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

            // Update connections. Detect new links.
            pKFi->UpdateConnections();
            LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
            for(vector<shared_ptr<KeyFrame> >::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
            {
                LoopConnections[pKFi].erase(*vit_prev);
            }
            for(vector<shared_ptr<KeyFrame> >::iterator vit2=vpCurrentConnectedKFs.begin(), vend2=vpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
            {
                LoopConnections[pKFi].erase(*vit2);
            }
        }

        // Optimize graph
        bool bFixedScale = mbFixScale;
        // TODO CHECK; Solo para el monocular inertial
        if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale=false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndFusion = std::chrono::steady_clock::now();

        double timeFusion = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndFusion - time_StartFusion).count();
        vdLoopFusion_ms.push_back(timeFusion);
#endif
        //cout << "Optimize essential graph" << endl;
        if(pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
        {
            Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
        }
        else
        {
            //cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
            Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
        }
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndOpt = std::chrono::steady_clock::now();

    double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOpt - time_EndFusion).count();
    vdLoopOptEss_ms.push_back(timeOptEss);
#endif

        mpAtlas->InformNewBigChange();

        // Add loop edge
        mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
        mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

        // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
        if(!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1))
        {
            mbRunningGBA = true;
            mbFinishedGBA = false;
            mbStopGBA = false;
            mnCorrectionGBA = mnNumCorrection;

            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
        }else
            mpMapSparsification->Release();

        // Loop closed. Release Local Mapping.
        mpLocalMapper->Release();
        mpTracker->SetLoopKeyFrame(mpLoopMatchedKF);

        mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
    }

    void LoopClosing::MergeLocal()
    {
        int numTemporalKFs = 25; //Temporal KFs in the local window if the map is inertial.

        //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
        shared_ptr<KeyFrame>  pNewChild;
        shared_ptr<KeyFrame>  pNewParent;

        vector<shared_ptr<KeyFrame> > vpLocalCurrentWindowKFs;
        vector<shared_ptr<KeyFrame> > vpMergeConnectedKFs;

        // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
        bool bRelaunchBA = false;

        //Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
        // If a Global Bundle Adjustment is running, abort it
        if(isRunningGBA())
        {
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if(mpThreadGBA)
            {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            bRelaunchBA = true;
        }

        //Verbose::PrintMess("MERGE-VISUAL: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
        //cout << "Request Stop Local Mapping" << endl;
        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }
        //cout << "Local Map stopped" << endl;

        mpLocalMapper->EmptyQueue();

        // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
        // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
        Map* pCurrentMap = mpCurrentKF->GetMap();
        Map* pMergeMap = mpMergeMatchedKF->GetMap();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif

        // Ensure current keyframe is updated
        mpCurrentKF->UpdateConnections();

        //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
        set<shared_ptr<KeyFrame> > spLocalWindowKFs;
        //Get MPs in the welding area from the current map
        set<shared_ptr<MapPoint> > spLocalWindowMPs;
        if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
        {
            shared_ptr<KeyFrame>  pKFi = mpCurrentKF;
            int nInserted = 0;
            while(pKFi && nInserted < numTemporalKFs)
            {
                spLocalWindowKFs.insert(pKFi);
                pKFi = mpCurrentKF->mPrevKF;
                nInserted++;

                set<shared_ptr<MapPoint> > spMPi = pKFi->GetMapPoints();
                spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
            }

            pKFi = mpCurrentKF->mNextKF;
            while(pKFi)
            {
                spLocalWindowKFs.insert(pKFi);

                set<shared_ptr<MapPoint> > spMPi = pKFi->GetMapPoints();
                spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

                pKFi = mpCurrentKF->mNextKF;
            }
        }
        else
        {
            spLocalWindowKFs.insert(mpCurrentKF);
        }

        vector<shared_ptr<KeyFrame> > vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
        spLocalWindowKFs.insert(mpCurrentKF);
        const int nMaxTries = 5;
        int nNumTries = 0;
        while(spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
        {
            vector<shared_ptr<KeyFrame> > vpNewCovKFs;
            vpNewCovKFs.empty();
            for(shared_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
            {
                vector<shared_ptr<KeyFrame> > vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
                for(shared_ptr<KeyFrame>  pKFcov : vpKFiCov)
                {
                    if(pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                    {
                        vpNewCovKFs.push_back(pKFcov);
                    }

                }
            }

            spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            nNumTries++;
        }

        for(shared_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
                continue;

            set<shared_ptr<MapPoint> > spMPs = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
        }

        //std::cout << "[Merge]: Ma = " << to_string(pCurrentMap->GetId()) << "; #KFs = " << to_string(spLocalWindowKFs.size()) << "; #MPs = " << to_string(spLocalWindowMPs.size()) << std::endl;

        set<shared_ptr<KeyFrame> > spMergeConnectedKFs;
        if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
        {
            shared_ptr<KeyFrame>  pKFi = mpMergeMatchedKF;
            int nInserted = 0;
            while(pKFi && nInserted < numTemporalKFs/2)
            {
                spMergeConnectedKFs.insert(pKFi);
                pKFi = mpCurrentKF->mPrevKF;
                nInserted++;
            }

            pKFi = mpMergeMatchedKF->mNextKF;
            while(pKFi && nInserted < numTemporalKFs)
            {
                spMergeConnectedKFs.insert(pKFi);
                pKFi = mpCurrentKF->mNextKF;
            }
        }
        else
        {
            spMergeConnectedKFs.insert(mpMergeMatchedKF);
        }
        vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
        nNumTries = 0;
        while(spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
        {
            vector<shared_ptr<KeyFrame> > vpNewCovKFs;
            for(shared_ptr<KeyFrame>  pKFi : spMergeConnectedKFs)
            {
                vector<shared_ptr<KeyFrame> > vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
                for(shared_ptr<KeyFrame>  pKFcov : vpKFiCov)
                {
                    if(pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                    {
                        vpNewCovKFs.push_back(pKFcov);
                    }

                }
            }

            spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            nNumTries++;
        }

        set<shared_ptr<MapPoint> > spMapPointMerge;
        for(shared_ptr<KeyFrame>  pKFi : spMergeConnectedKFs)
        {
            set<shared_ptr<MapPoint> > vpMPs = pKFi->GetMapPoints();
            spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
        }

        vector<shared_ptr<MapPoint> > vpCheckFuseMapPoint;
        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
        g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(),Twc.translation(),1.0);
        g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
        g2o::Sim3 g2oCorrectedScw = mg2oMergeScw; //TODO Check the transformation

        KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
        vCorrectedSim3[mpCurrentKF]=g2oCorrectedScw;
        vNonCorrectedSim3[mpCurrentKF]=g2oNonCorrectedScw;


#ifdef REGISTER_TIMES
        vnMergeKFs.push_back(spLocalWindowKFs.size() + spMergeConnectedKFs.size());
    vnMergeMPs.push_back(spLocalWindowMPs.size() + spMapPointMerge.size());
#endif
        for(shared_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
            {
                Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
                continue;
            }

            if(pKFi->GetMap() != pCurrentMap)
                Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

            g2o::Sim3 g2oCorrectedSiw;

            if(pKFi!=mpCurrentKF)
            {
                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
                //Pose without correction
                vNonCorrectedSim3[pKFi]=g2oSiw;

                Sophus::SE3d Tic = Tiw*Twc;
                g2o::Sim3 g2oSic(Tic.unit_quaternion(),Tic.translation(),1.0);
                g2oCorrectedSiw = g2oSic*mg2oMergeScw;
                vCorrectedSim3[pKFi]=g2oCorrectedSiw;
            }
            else
            {
                g2oCorrectedSiw = g2oCorrectedScw;
            }
            pKFi->mTcwMerge  = pKFi->GetPose();

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            double s = g2oCorrectedSiw.scale();
            pKFi->mfScale = s;
            Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

            pKFi->mTcwMerge = correctedTiw.cast<float>();

            if(pCurrentMap->isImuInitialized())
            {
                Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
            }

            //TODO DEBUG to know which are the KFs that had been moved to the other map
        }

        int numPointsWithCorrection = 0;

        //for(shared_ptr<MapPoint>  pMPi : spLocalWindowMPs)
        set<shared_ptr<MapPoint> >::iterator itMP = spLocalWindowMPs.begin();
        while(itMP != spLocalWindowMPs.end())
        {
            shared_ptr<MapPoint>  pMPi = *itMP;
            if(!pMPi || pMPi->isBad())
            {
                itMP = spLocalWindowMPs.erase(itMP);
                continue;
            }

            shared_ptr<KeyFrame>  pKFref = pMPi->GetReferenceKeyFrame();
            if(vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
            {
                itMP = spLocalWindowMPs.erase(itMP);
                numPointsWithCorrection++;
                continue;
            }
            g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            // Project with non-corrected pose and project back with corrected pose
            Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
            Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
            Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

            pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
            pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

            itMP++;
        }

        {
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            //std::cout << "Merge local window: " << spLocalWindowKFs.size() << std::endl;
            //std::cout << "[Merge]: init merging maps " << std::endl;
            for(shared_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
            {
                if(!pKFi || pKFi->isBad())
                {
                    //std::cout << "Bad KF in correction" << std::endl;
                    continue;
                }

                //std::cout << "KF id: " << pKFi->mnId << std::endl;

                pKFi->mTcwBefMerge = pKFi->GetPose();
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
                pKFi->SetPose(pKFi->mTcwMerge);

                // Make sure connections are updated
                pKFi->UpdateMap(pMergeMap);
                pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);

                if(pCurrentMap->isImuInitialized())
                {
                    pKFi->SetVelocity(pKFi->mVwbMerge);
                }
            }

            for(shared_ptr<MapPoint>  pMPi : spLocalWindowMPs)
            {
                if(!pMPi || pMPi->isBad())
                    continue;

                pMPi->SetWorldPos(pMPi->mPosMerge);
                pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }

            mpAtlas->ChangeMap(pMergeMap);
            mpAtlas->SetMapBad(pCurrentMap);
            pMergeMap->IncreaseChangeIndex();
            //TODO for debug
            pMergeMap->ChangeId(pCurrentMap->GetId());

            //std::cout << "[Merge]: merging maps finished" << std::endl;
        }

        //Rebuild the essential graph in the local window
        pCurrentMap->GetOriginKF()->SetFirstConnection(false);
        pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
        pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
        mpCurrentKF->ChangeParent(mpMergeMatchedKF);
        while(pNewChild)
        {
            pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
            shared_ptr<KeyFrame>  pOldParent = pNewChild->GetParent();

            pNewChild->ChangeParent(pNewParent);

            pNewParent = pNewChild;
            pNewChild = pOldParent;

        }

        //Update the connections between the local window
        mpMergeMatchedKF->UpdateConnections();

        vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
        //vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        //std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        // Project MapPoints observed in the neighborhood of the merge keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        //std::cout << "[Merge]: start fuse points" << std::endl;
        SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);
        //std::cout << "[Merge]: fuse points finished" << std::endl;

        // Update connectivity
        for(shared_ptr<KeyFrame>  pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }
        for(shared_ptr<KeyFrame>  pKFi : spMergeConnectedKFs)
        {
            if(!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }

        //std::cout << "[Merge]: Start welding bundle adjustment" << std::endl;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartWeldingBA = std::chrono::steady_clock::now();

    double timeMergeMaps = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_StartWeldingBA - time_StartMerge).count();
    vdMergeMaps_ms.push_back(timeMergeMaps);
#endif

        bool bStop = false;
        vpLocalCurrentWindowKFs.clear();
        vpMergeConnectedKFs.clear();
        std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
        std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
        if (mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
        {
            Optimizer::MergeInertialBA(mpCurrentKF,mpMergeMatchedKF,&bStop, pCurrentMap,vCorrectedSim3);
        }
        else
        {
            Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs,&bStop);
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndWeldingBA = std::chrono::steady_clock::now();

    double timeWeldingBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndWeldingBA - time_StartWeldingBA).count();
    vdWeldingBA_ms.push_back(timeWeldingBA);
#endif
        //std::cout << "[Merge]: Welding bundle adjustment finished" << std::endl;

        // Loop closed. Release Local Mapping.
        mpLocalMapper->Release();

        //Update the non critical area from the current map to the merged map
        vector<shared_ptr<KeyFrame> > vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
        vector<shared_ptr<MapPoint> > vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

        if(vpCurrentMapKFs.size() == 0){}
        else {
            if(mpTracker->mSensor == System::MONOCULAR)
            {
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

                for(shared_ptr<KeyFrame>  pKFi : vpCurrentMapKFs)
                {
                    if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }

                    g2o::Sim3 g2oCorrectedSiw;

                    Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
                    //Pose without correction
                    vNonCorrectedSim3[pKFi]=g2oSiw;

                    Sophus::SE3d Tic = Tiw*Twc;
                    g2o::Sim3 g2oSim(Tic.unit_quaternion(),Tic.translation(),1.0);
                    g2oCorrectedSiw = g2oSim*mg2oMergeScw;
                    vCorrectedSim3[pKFi]=g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    double s = g2oCorrectedSiw.scale();

                    pKFi->mfScale = s;

                    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / s);

                    pKFi->mTcwBefMerge = pKFi->GetPose();
                    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                    pKFi->SetPose(correctedTiw.cast<float>());

                    if(pCurrentMap->isImuInitialized())
                    {
                        Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                        pKFi->SetVelocity(Rcor * pKFi->GetVelocity()); // TODO: should add here scale s
                    }

                }
                for(shared_ptr<MapPoint>  pMPi : vpCurrentMapMPs)
                {
                    if(!pMPi || pMPi->isBad()|| pMPi->GetMap() != pCurrentMap)
                        continue;

                    shared_ptr<KeyFrame>  pKFref = pMPi->GetReferenceKeyFrame();
                    g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                    // Project with non-corrected pose and project back with corrected pose
                    Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                    Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
                    pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

                    pMPi->UpdateNormalAndDepth();
                }
            }

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            // Optimize graph (and update the loop position for each element form the begining to the end)
            if(mpTracker->mSensor != System::MONOCULAR)
            {
                Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
            }


            {
                // Get Merge Map Mutex
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
                unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

                //std::cout << "Merge outside KFs: " << vpCurrentMapKFs.size() << std::endl;
                for(shared_ptr<KeyFrame>  pKFi : vpCurrentMapKFs)
                {
                    if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }
                    //std::cout << "KF id: " << pKFi->mnId << std::endl;

                    // Make sure connections are updated
                    pKFi->UpdateMap(pMergeMap);
                    pMergeMap->AddKeyFrame(pKFi);
                    pCurrentMap->EraseKeyFrame(pKFi);
                }

                for(shared_ptr<MapPoint>  pMPi : vpCurrentMapMPs)
                {
                    if(!pMPi || pMPi->isBad())
                        continue;

                    pMPi->UpdateMap(pMergeMap);
                    pMergeMap->AddMapPoint(pMPi);
                    pCurrentMap->EraseMapPoint(pMPi);
                }
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndOptEss = std::chrono::steady_clock::now();

    double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOptEss - time_EndWeldingBA).count();
    vdMergeOptEss_ms.push_back(timeOptEss);
#endif


        mpLocalMapper->Release();

        if(bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1)))
        {
            // Launch a new thread to perform Global Bundle Adjustment
            mbRunningGBA = true;
            mbFinishedGBA = false;
            mbStopGBA = false;
            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this, pMergeMap, mpCurrentKF->mnId);
        }

        mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
        mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

        pCurrentMap->IncreaseChangeIndex();
        pMergeMap->IncreaseChangeIndex();

        mpAtlas->RemoveBadMaps();

    }


    void LoopClosing::MergeLocal2()
    {
        //cout << "Merge detected!!!!" << endl;

        int numTemporalKFs = 11; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

        //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
        shared_ptr<KeyFrame>  pNewChild;
        shared_ptr<KeyFrame>  pNewParent;

        vector<shared_ptr<KeyFrame> > vpLocalCurrentWindowKFs;
        vector<shared_ptr<KeyFrame> > vpMergeConnectedKFs;

        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

        // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
        bool bRelaunchBA = false;

        //cout << "Check Full Bundle Adjustment" << endl;
        // If a Global Bundle Adjustment is running, abort it
        if(isRunningGBA())
        {
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if(mpThreadGBA)
            {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            bRelaunchBA = true;
        }


        //cout << "Request Stop Local Mapping" << endl;
        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }
        //cout << "Local Map stopped" << endl;

        Map* pCurrentMap = mpCurrentKF->GetMap();
        Map* pMergeMap = mpMergeMatchedKF->GetMap();

        {
            float s_on = mSold_new.scale();
            Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

            //cout << "KFs before empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;
            mpLocalMapper->EmptyQueue();
            //cout << "KFs after empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            //cout << "updating active map to merge reference" << endl;
            //cout << "curr merge KF id: " << mpCurrentKF->mnId << endl;
            //cout << "curr tracking KF id: " << mpTracker->GetLastKeyFrame()->mnId << endl;
            bool bScaleVel=false;
            if(s_on!=1)
                bScaleVel=true;
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on,s_on,bScaleVel);
            mpTracker->UpdateFrameIMU(s_on,mpCurrentKF->GetImuBias(),mpTracker->GetLastKeyFrame());

            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        }

        const int numKFnew=pCurrentMap->KeyFramesInMap();

        if((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
           && !pCurrentMap->GetIniertialBA2()){
            // Map is not completly initialized
            Eigen::Vector3d bg, ba;
            bg << 0., 0., 0.;
            ba << 0., 0., 0.;
            Optimizer::InertialOptimization(pCurrentMap,bg,ba);
            IMU::Bias b (ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);
            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
            mpTracker->UpdateFrameIMU(1.0f,b,mpTracker->GetLastKeyFrame());

            // Set map initialized
            pCurrentMap->SetIniertialBA2();
            pCurrentMap->SetIniertialBA1();
            pCurrentMap->SetImuInitialized();

        }


        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        // Load KFs and MPs from merge map
        //cout << "updating current map" << endl;
        {
            // Get Merge Map Mutex (This section stops tracking!!)
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map


            vector<shared_ptr<KeyFrame> > vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
            vector<shared_ptr<MapPoint> > vpMergeMapMPs = pMergeMap->GetAllMapPoints();


            for(shared_ptr<KeyFrame>  pKFi : vpMergeMapKFs)
            {
                if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
                {
                    continue;
                }

                // Make sure connections are updated
                pKFi->UpdateMap(pCurrentMap);
                pCurrentMap->AddKeyFrame(pKFi);
                pMergeMap->EraseKeyFrame(pKFi);
            }

            for(shared_ptr<MapPoint>  pMPi : vpMergeMapMPs)
            {
                if(!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                    continue;

                pMPi->UpdateMap(pCurrentMap);
                pCurrentMap->AddMapPoint(pMPi);
                pMergeMap->EraseMapPoint(pMPi);
            }

            // Save non corrected poses (already merged maps)
            vector<shared_ptr<KeyFrame> > vpKFs = pCurrentMap->GetAllKeyFrames();
            for(shared_ptr<KeyFrame>  pKFi : vpKFs)
            {
                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
                NonCorrectedSim3[pKFi]=g2oSiw;
            }
        }

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        //cout << "end updating current map" << endl;

        // Critical zone
        //bool good = pCurrentMap->CheckEssentialGraph();
        /*if(!good)
            cout << "BAD ESSENTIAL GRAPH!!" << endl;*/

        //cout << "Update essential graph" << endl;
        // mpCurrentKF->UpdateConnections(); // to put at false mbFirstConnection
        pMergeMap->GetOriginKF()->SetFirstConnection(false);
        pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
        pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
        mpMergeMatchedKF->ChangeParent(mpCurrentKF);
        while(pNewChild)
        {
            pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
            shared_ptr<KeyFrame>  pOldParent = pNewChild->GetParent();
            pNewChild->ChangeParent(pNewParent);
            pNewParent = pNewChild;
            pNewChild = pOldParent;

        }


        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        //cout << "end update essential graph" << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 1!!" << endl;*/

        //cout << "Update relationship between KFs" << endl;
        vector<shared_ptr<MapPoint> > vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
        vector<shared_ptr<KeyFrame> > vpCurrentConnectedKFs;

        vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
        vector<shared_ptr<KeyFrame> > aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        vpMergeConnectedKFs.insert(vpMergeConnectedKFs.end(), aux.begin(), aux.end());
        if (vpMergeConnectedKFs.size()>6)
            vpMergeConnectedKFs.erase(vpMergeConnectedKFs.begin()+6,vpMergeConnectedKFs.end());
        /*mvpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);*/

        mpCurrentKF->UpdateConnections();
        vpCurrentConnectedKFs.push_back(mpCurrentKF);
        /*vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurrentConnectedKFs.push_back(mpCurrentKF);*/
        aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
        if (vpCurrentConnectedKFs.size()>6)
            vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin()+6,vpCurrentConnectedKFs.end());

        set<shared_ptr<MapPoint> > spMapPointMerge;
        for(shared_ptr<KeyFrame>  pKFi : vpMergeConnectedKFs)
        {
            set<shared_ptr<MapPoint> > vpMPs = pKFi->GetMapPoints();
            spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
            if(spMapPointMerge.size()>1000)
                break;
        }

        /*cout << "vpCurrentConnectedKFs.size() " << vpCurrentConnectedKFs.size() << endl;
        cout << "mvpMergeConnectedKFs.size() " << mvpMergeConnectedKFs.size() << endl;
        cout << "spMapPointMerge.size() " << spMapPointMerge.size() << endl;*/


        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));
        //cout << "Finished to update relationship between KFs" << endl;

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 2!!" << endl;*/

        //cout << "start SearchAndFuse" << endl;
        SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);
        //cout << "end SearchAndFuse" << endl;

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 3!!" << endl;

        cout << "Init to update connections" << endl;*/


        for(shared_ptr<KeyFrame>  pKFi : vpCurrentConnectedKFs)
        {
            if(!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }
        for(shared_ptr<KeyFrame>  pKFi : vpMergeConnectedKFs)
        {
            if(!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }
        //cout << "end update connections" << endl;

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 4!!" << endl;*/

        // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
        if (numKFnew<10){
            mpLocalMapper->Release();
            return;
        }

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 5!!" << endl;*/

        // Perform BA
        bool bStopFlag=false;
        shared_ptr<KeyFrame>  pCurrKF = mpTracker->GetLastKeyFrame();
        //cout << "start MergeInertialBA" << endl;
        Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap,CorrectedSim3);
        //cout << "end MergeInertialBA" << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 6!!" << endl;*/

        // Release Local Mapping.
        mpLocalMapper->Release();


        return;
    }

    void LoopClosing::CheckObservations(set<shared_ptr<KeyFrame> > &spKFsMap1, set<shared_ptr<KeyFrame> > &spKFsMap2)
    {
        cout << "----------------------" << endl;
        for(shared_ptr<KeyFrame>  pKFi1 : spKFsMap1)
        {
            map<shared_ptr<KeyFrame> , int> mMatchedMP;
            set<shared_ptr<MapPoint> > spMPs = pKFi1->GetMapPoints();

            for(shared_ptr<MapPoint>  pMPij : spMPs)
            {
                if(!pMPij || pMPij->isBad())
                {
                    continue;
                }

                map<shared_ptr<KeyFrame> , tuple<int,int>> mMPijObs = pMPij->GetObservations();
                for(shared_ptr<KeyFrame>  pKFi2 : spKFsMap2)
                {
                    if(mMPijObs.find(pKFi2) != mMPijObs.end())
                    {
                        if(mMatchedMP.find(pKFi2) != mMatchedMP.end())
                        {
                            mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                        }
                        else
                        {
                            mMatchedMP[pKFi2] = 1;
                        }
                    }
                }

            }

            if(mMatchedMP.size() == 0)
            {
                cout << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
            }
            else
            {
                cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with " << mMatchedMP.size() << " KF from the other map" << endl;
                for(pair<shared_ptr<KeyFrame> , int> matchedKF : mMatchedMP)
                {
                    cout << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
                }
            }
        }
        cout << "----------------------" << endl;
    }


    void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<shared_ptr<MapPoint> > &vpMapPoints)
    {
        ORBmatcher matcher(0.8);

        int total_replaces = 0;

        //cout << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" << endl;
        //cout << "FUSE: Intially there are " << CorrectedPosesMap.size() << " KFs" << endl;
        for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
        {
            int num_replaces = 0;
            shared_ptr<KeyFrame>  pKFi = mit->first;
            Map* pMap = pKFi->GetMap();

            g2o::Sim3 g2oScw = mit->second;
            Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

            vector<shared_ptr<MapPoint> > vpReplacePoints(vpMapPoints.size(),static_cast<shared_ptr<MapPoint> >(NULL));
            int numFused = matcher.Fuse(pKFi,Scw,vpMapPoints,4,vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            const int nLP = vpMapPoints.size();
            for(int i=0; i<nLP;i++)
            {
                shared_ptr<MapPoint>  pRep = vpReplacePoints[i];
                if(pRep)
                {


                    num_replaces += 1;
                    pRep->Replace(vpMapPoints[i]);

                }
            }

            total_replaces += num_replaces;
        }
        //cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
    }


    void LoopClosing::SearchAndFuse(const vector<shared_ptr<KeyFrame> > &vConectedKFs, vector<shared_ptr<MapPoint> > &vpMapPoints)
    {
        ORBmatcher matcher(0.8);

        int total_replaces = 0;

        //cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
        //cout << "FUSE-POSE: Intially there are " << vConectedKFs.size() << " KFs" << endl;
        for(auto mit=vConectedKFs.begin(), mend=vConectedKFs.end(); mit!=mend;mit++)
        {
            int num_replaces = 0;
            shared_ptr<KeyFrame>  pKF = (*mit);
            Map* pMap = pKF->GetMap();
            Sophus::SE3f Tcw = pKF->GetPose();
            Sophus::Sim3f Scw(Tcw.unit_quaternion(),Tcw.translation());
            Scw.setScale(1.f);
            /*std::cout << "These should be zeros: " <<
                Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
                Scw.translation() - Tcw.translation() << std::endl <<
                Scw.scale() - 1.f << std::endl;*/
            vector<shared_ptr<MapPoint> > vpReplacePoints(vpMapPoints.size(),static_cast<shared_ptr<MapPoint> >(NULL));
            matcher.Fuse(pKF,Scw,vpMapPoints,4,vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            const int nLP = vpMapPoints.size();
            for(int i=0; i<nLP;i++)
            {
                shared_ptr<MapPoint>  pRep = vpReplacePoints[i];
                if(pRep)
                {
                    num_replaces += 1;
                    pRep->Replace(vpMapPoints[i]);
                }
            }
            /*cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << " MPs fused" << endl;
            total_replaces += num_replaces;*/
        }
        //cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
    }



    void LoopClosing::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetRequested)
                    break;
            }
            usleep(5000);
        }
    }

    void LoopClosing::RequestResetActiveMap(Map *pMap)
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetActiveMapRequested = true;
            mpMapToReset = pMap;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetActiveMapRequested)
                    break;
            }
            usleep(3000);
        }
    }

    void LoopClosing::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            cout << "Loop closer reset requested..." << endl;
            mlpLoopKeyFrameQueue.clear();
            mLastLoopKFid=0;  //TODO old variable, it is not use in the new algorithm
            mbResetRequested=false;
            mbResetActiveMapRequested = false;
        }
        else if(mbResetActiveMapRequested)
        {

            for (list<shared_ptr<KeyFrame> >::const_iterator it=mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
            {
                shared_ptr<KeyFrame>  pKFi = *it;
                if(pKFi->GetMap() == mpMapToReset)
                {
                    it = mlpLoopKeyFrameQueue.erase(it);
                }
                else
                    ++it;
            }

            mLastLoopKFid=mpAtlas->GetLastInitKFid(); //TODO old variable, it is not use in the new algorithm
            mbResetActiveMapRequested=false;

        }
    }

    void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF)
    {
        Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();

    nFGBA_exec += 1;

    vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
    vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

        const bool bImuInit = pActiveMap->isImuInitialized();

        if(!bImuInit)
            Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false);
        else
            Optimizer::FullInertialBA(pActiveMap,7,false,nLoopKF,&mbStopGBA);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndGBA = std::chrono::steady_clock::now();

    double timeGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndGBA - time_StartFGBA).count();
    vdGBA_ms.push_back(timeGBA);

    if(mbStopGBA)
    {
        nFGBA_abort += 1;
    }
#endif

        int idx =  mnFullBAIdx;
        // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

        // Update all MapPoints and KeyFrames
        // Local Mapping was active during BA, that means that there might be new keyframes
        // not included in the Global BA and they are not consistent with the updated map.
        // We need to propagate the correction through the spanning tree
        {
            unique_lock<mutex> lock(mMutexGBA);
            if(idx!=mnFullBAIdx)
                return;

            if(!bImuInit && pActiveMap->isImuInitialized())
                return;

            if(!mbStopGBA)
            {
                Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
                Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);
                cout << "Global Bundle Adjustment finished" << endl;
                cout << "Updating map ..." << endl;
                mpLocalMapper->RequestStop();
//                mpMapSaprsification->RequestStop();
                // Wait until Local Mapping has effectively stopped

                while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
                {
                    usleep(1000);
                }

                // Get Map Mutex
                unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
                // cout << "LC: Update Map Mutex adquired" << endl;

                //pActiveMap->PrintEssentialGraph();
                // Correct keyframes starting at map first keyframe
                list<shared_ptr<KeyFrame> > lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),pActiveMap->mvpKeyFrameOrigins.end());

                while(!lpKFtoCheck.empty())
                {
                    shared_ptr<KeyFrame>  pKF = lpKFtoCheck.front();
                    const set<shared_ptr<KeyFrame> > sChilds = pKF->GetChilds();
                    //cout << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                    //cout << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                    Sophus::SE3f Twc = pKF->GetPoseInverse();
                    //cout << "Twc: " << Twc << endl;
                    //cout << "GBA: Correct KeyFrames" << endl;
                    for(set<shared_ptr<KeyFrame> >::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                    {
                        shared_ptr<KeyFrame>  pChild = *sit;
                        if(!pChild || pChild->isBad())
                            continue;

                        if(pChild->mnBAGlobalForKF!=nLoopKF)
                        {
                            //cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                            //cout << " child id: " << pChild->mnId << endl;
                            Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                            //cout << "Child pose: " << Tchildc << endl;
                            //cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;

                            Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                            if(pChild->isVelocitySet()){
                                pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                            }
                            else
                                Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                            //cout << "Child bias: " << pChild->GetImuBias() << endl;
                            pChild->mBiasGBA = pChild->GetImuBias();


                            pChild->mnBAGlobalForKF = nLoopKF;

                        }
                        lpKFtoCheck.push_back(pChild);
                    }

                    //cout << "-------Update pose" << endl;
                    pKF->mTcwBefGBA = pKF->GetPose();
                    //cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                    pKF->SetPose(pKF->mTcwGBA);
                    /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
                    cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
                    double dist = cv::norm(trasl);
                    cout << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" << endl;
                    double desvX = 0;
                    double desvY = 0;
                    double desvZ = 0;
                    if(pKF->mbHasHessian)
                    {
                        cv::Mat hessianInv = pKF->mHessianPose.inv();

                        double covX = hessianInv.at<double>(3,3);
                        desvX = std::sqrt(covX);
                        double covY = hessianInv.at<double>(4,4);
                        desvY = std::sqrt(covY);
                        double covZ = hessianInv.at<double>(5,5);
                        desvZ = std::sqrt(covZ);
                        pKF->mbHasHessian = false;
                    }
                    if(dist > 1)
                    {
                        cout << "--To much distance correction: It has " << pKF->GetConnectedKeyFrames().size() << " connected KFs" << endl;
                        cout << "--It has " << pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" << endl;
                        cout << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected KF with 50 common matches or more" << endl;
                        cout << "--It has " << pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" << endl;

                        cout << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ << endl;


                        string strNameFile = pKF->mNameFile;
                        cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);

                        cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);

                        vector<shared_ptr<MapPoint> > vpMapPointsKF = pKF->GetMapPointMatches();
                        int num_MPs = 0;
                        for(int i=0; i<vpMapPointsKF.size(); ++i)
                        {
                            if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                            {
                                continue;
                            }
                            num_MPs += 1;
                            string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                            cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                            cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                        }
                        cout << "--It has " << num_MPs << " MPs matched in the map" << endl;

                        string namefile = "./test_GBA/GBA_" + to_string(nLoopKF) + "_KF" + to_string(pKF->mnId) +"_D" + to_string(dist) +".png";
                        cv::imwrite(namefile, imLeft);
                    }*/


                    if(pKF->bImu)
                    {
                        //cout << "-------Update inertial values" << endl;
                        pKF->mVwbBefGBA = pKF->GetVelocity();
                        //if (pKF->mVwbGBA.empty())
                        //    Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                        //assert(!pKF->mVwbGBA.empty());
                        pKF->SetVelocity(pKF->mVwbGBA);
                        pKF->SetNewBias(pKF->mBiasGBA);
                    }

                    lpKFtoCheck.pop_front();
                }

                //cout << "GBA: Correct MapPoints" << endl;
                // Correct MapPoints
                const vector<shared_ptr<MapPoint> > vpMPs = pActiveMap->GetAllMapPoints();

                for(size_t i=0; i<vpMPs.size(); i++)
                {
                    shared_ptr<MapPoint>  pMP = vpMPs[i];

                    if(pMP->isBad())
                        continue;

                    if(pMP->mnBAGlobalForKF==nLoopKF)
                    {
                        // If optimized by Global BA, just update
                        pMP->SetWorldPos(pMP->mPosGBA);
                    }
                    else
                    {
                        // Update according to the correction of its reference keyframe
                        shared_ptr<KeyFrame>  pRefKF = pMP->GetReferenceKeyFrame();

                        if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                            continue;

                        /*if(pRefKF->mTcwBefGBA.empty())
                            continue;*/

                        // Map to non-corrected camera
                        // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                        // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                        Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                        // Backproject using corrected camera
                        pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                    }
                }

                pActiveMap->InformNewBigChange();
                pActiveMap->IncreaseChangeIndex();

                // TODO Check this update
                // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

                mpLocalMapper->Release();
                mpMapSparsification->Release();
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();

            double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateMap - time_EndGBA).count();
            vdUpdateMap_ms.push_back(timeUpdateMap);

            double timeFGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateMap - time_StartFGBA).count();
            vdFGBATotal_ms.push_back(timeFGBA);
#endif
                cout << "Map updated!" << endl;
                Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
            }

            mbFinishedGBA = true;
            mbRunningGBA = false;
        }
    }

    void LoopClosing::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        // cout << "LC: Finish requested" << endl;
        mbFinishRequested = true;
    }

    bool LoopClosing::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LoopClosing::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool LoopClosing::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

} //namespace ORB_SLAM