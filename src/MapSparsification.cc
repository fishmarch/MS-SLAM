#include "MapSparsification.h"

namespace ORB_SLAM3 {
    MapSparsification::MapSparsification(const string &strSettingsFile,
                                         Atlas *pAtlas, bool bInertial) :
            mbStopRequested(false), mbStopped(true), mnId(0), mGRBEnv(GRBEnv(true)),
            mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), mbInertial(bInertial) {
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        mnMinNum = fsSettings["Sparsification.N"];
        mfLambda = fsSettings["Sparsification.Lambda"];
        mfGridLambda = fsSettings["Sparsification.GridLambda"];
        mnWindowLength = fsSettings["Sparsification.WindowLength"];
        cout << endl << "*****************************************" << endl;
        cout << "Map Sparsification settings: " << endl;
        cout << "Sparsification.N: " << mnMinNum << endl;
        cout << "Sparsification.Lambda: " << mfLambda << endl;
        cout << "Sparsification.GridLambda: " << mfGridLambda << endl;
        cout << "Sparsification.WindowLength: " << mnWindowLength << endl;
        cout << "*****************************************" << endl;
        mGRBEnv.start();
    }

    void MapSparsification::Run() {
        mbFinished = false;
        while (true) {
            if (CheckNewKeyFrames()) {
                {
                    unique_lock<mutex> lock2(mMutexStop);
                    mbStopped = false;
                }
                vector<shared_ptr<KeyFrame> > vpKFs = GetLastestKeyFrames();
                Sparsifying(vpKFs);
                {
                    unique_lock<mutex> lock2(mMutexStop);
                    mbStopped = true;
                }
            }
            if (CheckFinish()) {

                vector<shared_ptr<KeyFrame>> vAllKeyFrames = mpAtlas->GetAllKeyFrames();
                vector<shared_ptr<KeyFrame>> vRemainKeyFrames;
                for (shared_ptr<KeyFrame> pKFi: vAllKeyFrames) {
                    if (!pKFi->mbSparsified)
                        vRemainKeyFrames.push_back(pKFi);
                }
                Sparsifying(vRemainKeyFrames);
                for (shared_ptr<KeyFrame> pKFi: vRemainKeyFrames) {
                    pKFi->EraseBadDescriptor();
                }

                break;
            }
            usleep(3000);
        }
        SetFinish();
    }

    void MapSparsification::Sparsifying(vector<shared_ptr<KeyFrame> > &vpKFs) {
        vector<shared_ptr<KeyFrame> > vAllKFs =  mpAtlas->GetAllKeyFrames();
        mnId++;
        GRBModel model = GRBModel(mGRBEnv);
        std::vector<GRBVar> vxs; // all parameters for every MP
        GRBLinExpr expr = 0; // cost function
        vector<shared_ptr<MapPoint> > vLocalMapPoints;
        long unsigned int index = 0;
        int nMaxObsevation = 0;
        for (shared_ptr<KeyFrame> pKFi: vpKFs) {
            vector<shared_ptr<MapPoint>> vMPs = pKFi->GetMapPointMatches();
            for (shared_ptr<MapPoint> pMPi: vMPs) {
                if (!pMPi || pMPi->isBad())
                    continue;
                int nObservation = pMPi->Observations();
                if (nObservation > nMaxObsevation)
                    nMaxObsevation = nObservation;
            }
        }

        for (auto lit = vpKFs.begin(), lend = vpKFs.end(); lit != lend; lit++) {
            GRBLinExpr expr_cons = 0;
            std::vector<std::vector<std::vector<size_t> > > grids = (*lit)->GetFeatureGrids();
            (*lit)->mnMapSaprsificationId = mnId;
            for (vector<vector<size_t>> &gridi: grids) {
                for (vector<size_t> &grid: gridi) {
                    if (grid.empty())
                        continue;
                    GRBLinExpr expr_cons_grid = 0;
                    bool bValidCell = false;
                    for (size_t i: grid) {
                        shared_ptr<MapPoint> pMP = (*lit)->GetMapPoint(i);
                        if (pMP && (!pMP->isBad())) {
                            if (pMP->mnMapSparsificationId != mnId) {
                                vLocalMapPoints.push_back(pMP);
                                pMP->mnMapSparsificationId = mnId;
                                GRBVar x = model.addVar(0, 1, 0, GRB_BINARY);
                                float coef = (nMaxObsevation - pMP->Observations());
                                expr += coef * x;
                                vxs.push_back(x);
                                pMP->mnIndexForSparsification = index;
                                index++;
                                expr_cons += x;
                                expr_cons_grid += x;
                                bValidCell = true;
                            } else {
                                long unsigned int iForComp = pMP->mnIndexForSparsification;
                                expr_cons += vxs[iForComp];
                                expr_cons_grid += vxs[iForComp];
                                bValidCell = true;
                            }
                        }
                    }
                    if (bValidCell) {
                        GRBVar th_grid = model.addVar(0, 1, 0, GRB_BINARY);
                        expr_cons_grid += th_grid;
                        model.addConstr(expr_cons_grid, '>', 1);
                        expr += mfGridLambda * th_grid;
                    }
                }
            }
            GRBVar th = model.addVar(0, 1000, 0, GRB_INTEGER);
            expr += mfLambda * th;
            expr_cons += th;
            model.addConstr(expr_cons, '>', mnMinNum);
        }

        map<shared_ptr<KeyFrame>, int> extraNum;
        map<shared_ptr<KeyFrame>, GRBLinExpr> extraConstrints;
        for (auto pMPi: vLocalMapPoints) {
            long unsigned int iForComp = pMPi->mnIndexForSparsification;
            const map<shared_ptr<KeyFrame>, tuple<int, int>> observations = pMPi->GetObservations();
            for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
                shared_ptr<KeyFrame> pKFi = mit->first;
                if (pKFi->mnMapSaprsificationId != mnId) {
                    if (extraNum.count(pKFi)) {
                        extraNum[pKFi]++;
                        extraConstrints[pKFi] += vxs[iForComp];
                    } else {
                        extraNum[pKFi] = 1;
                        extraConstrints[pKFi] = vxs[iForComp];
                    }
                }
            }
        }

        for (auto it = extraNum.begin(), itend = extraNum.end(); it != itend; it++) {
            shared_ptr<KeyFrame> pKFi = it->first;
            float nTotal = pKFi->GetNumberMPs();
            float nMini = (float) it->second / nTotal * mnMinNum;
            GRBVar th = model.addVar(0, 1000, 0, GRB_INTEGER);
            expr += mfLambda * th;
            model.addConstr(extraConstrints[pKFi] + th, '>', nMini);
        }

        model.setObjective(expr, GRB_MINIMIZE);
        model.set(GRB_IntParam_OutputFlag, 0);
        float MIPGap = 0.0020;
        model.set(GRB_DoubleParam_MIPGap, MIPGap);
        model.optimize();

        for (int i = 0; i < vLocalMapPoints.size(); ++i) {
            shared_ptr<MapPoint> pMPi = vLocalMapPoints[i];
            long unsigned int iForComp = pMPi->mnIndexForSparsification;
            double keep = vxs[iForComp].get(GRB_DoubleAttr_X);
            if (keep <= 0) {
                pMPi->SetBadFlag();
            }
        }

        for (auto pKFi: vpKFs) {
            mpLoopClosing->InsertSparsifiedKeyFrame(pKFi);
        }
    }

    vector<shared_ptr<KeyFrame> > MapSparsification::GetLastestKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKFs);
        if (mvpNewKeyFrames.size() > mnWindowLength) {
            vector<shared_ptr<KeyFrame> > vKFs = vector<shared_ptr<KeyFrame> >(mvpNewKeyFrames.begin(),
                                                                               mvpNewKeyFrames.begin() +
                                                                               mnWindowLength);
            mvpNewKeyFrames.erase(mvpNewKeyFrames.begin(), mvpNewKeyFrames.begin() + mnWindowLength);
            return vKFs;
        } else {
            vector<shared_ptr<KeyFrame> > vKFs = vector<shared_ptr<KeyFrame> >(mvpNewKeyFrames.begin(),
                                                                               mvpNewKeyFrames.end());
            mvpNewKeyFrames.clear();
            return vKFs;
        }
    }

    void MapSparsification::InsertKeyFrame(shared_ptr<KeyFrame> pKF) {
        unique_lock<mutex> lock(mMutexNewKFs);
        mvpNewKeyFrames.push_back(pKF);
    }

    bool MapSparsification::CheckNewKeyFrames() {
        unique_lock<mutex> lock2(mMutexStop);
        unique_lock<mutex> lock(mMutexNewKFs);
        return mvpNewKeyFrames.size() > 10 && (!mbStopRequested) &&
               (!mbInertial || mvpNewKeyFrames[0]->GetMap()->GetIniertialBA2());
    }

    void MapSparsification::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

    void MapSparsification::RequestStop() {
        unique_lock<mutex> lock2(mMutexStop);
        mbStopRequested = true;
        cout << "Map Sparsification STOP" << endl;
    }

    void MapSparsification::Release() {
        unique_lock<mutex> lock2(mMutexStop);
        mbStopRequested = false;
        cout << "Map Sparsification RELEASE" << endl;
    }

    bool MapSparsification::isStopped() {
        unique_lock<mutex> lock2(mMutexStop);
        return mbStopped;
    }

    void MapSparsification::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool MapSparsification::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void MapSparsification::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool MapSparsification::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

}