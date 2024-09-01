#ifndef ORB_SLAM3_MAPSPARSIFICATION_H
#define ORB_SLAM3_MAPSPARSIFICATION_H

#include "KeyFrame.h"
#include "Map.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "gurobi_c++.h"

#include <mutex>
#include <malloc.h>

namespace ORB_SLAM3 {
    class Map;

    class LocalMapping;

    class System;

    class LoopClosing;

    class Atlas;

    class MapSparsification {
    public:
        MapSparsification(const string &strSettingsFile, Atlas* pAtlas, bool bInertial);

        void Run();

        bool CheckNewKeyFrames();

        void InsertKeyFrame(shared_ptr<KeyFrame> pKF);

        void SetLoopClosing(LoopClosing *pLoopClosing);

        vector<shared_ptr<KeyFrame> > GetLastestKeyFrames();

        bool isStopped();
        void RequestStop();
        void Release();
        void RequestFinish();
        bool isFinished();
        int mnMinNum;
    private:
        void Sparsifying(vector<shared_ptr<KeyFrame> > &vpKFs);
        bool CheckFinish();
        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        long unsigned int mnId;
        bool mbStopRequested;
        bool mbStopped;

        GRBEnv mGRBEnv;
        float mfLambda;
        float mfGridLambda;
        int mnWindowLength;
        std::vector<shared_ptr<KeyFrame> > mvpNewKeyFrames;
        std::mutex mMutexNewKFs;
        std::mutex mMutexStop;
        LoopClosing *mpLoopClosing;
        shared_ptr<KeyFrame> mpKFTest;
        Atlas* mpAtlas;
        bool mbInertial;
    };
}

#endif //ORB_SLAM3_MAPSPARSIFICATION_H
