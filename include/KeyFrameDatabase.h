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


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include<mutex>


namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class Map;


class KeyFrameDatabase
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mvBackupInvertedFileId;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase(){}
    KeyFrameDatabase(const ORBVocabulary &voc);

    void add(shared_ptr<KeyFrame>  pKF);

    void erase(shared_ptr<KeyFrame>  pKF);

    void clear();
    void clearMap(Map* pMap);

    // Loop Detection(DEPRECATED)
    std::vector<shared_ptr<KeyFrame> > DetectLoopCandidates(shared_ptr<KeyFrame>  pKF, float minScore);

    // Loop and Merge Detection
    void DetectCandidates(shared_ptr<KeyFrame>  pKF, float minScore,vector<shared_ptr<KeyFrame> >& vpLoopCand, vector<shared_ptr<KeyFrame> >& vpMergeCand);
    void DetectBestCandidates(shared_ptr<KeyFrame> pKF, vector<shared_ptr<KeyFrame> > &vpLoopCand, vector<shared_ptr<KeyFrame> > &vpMergeCand, int nMinWords);
    void DetectNBestCandidates(shared_ptr<KeyFrame> pKF, vector<shared_ptr<KeyFrame> > &vpLoopCand, vector<shared_ptr<KeyFrame> > &vpMergeCand, int nNumCandidates);

    // Relocalization
    std::vector<shared_ptr<KeyFrame> > DetectRelocalizationCandidates(Frame* F, Map* pMap);

    void PreSave();
    void PostLoad(map<long unsigned int, shared_ptr<KeyFrame> > mpKFid);
    void SetORBVocabulary(ORBVocabulary* pORBVoc);

protected:

   // Associated vocabulary
   const ORBVocabulary* mpVoc;

   // Inverted file
   std::vector<list<shared_ptr<KeyFrame> > > mvInvertedFile;

   // For save relation without pointer, this is necessary for save/load function
   std::vector<list<long unsigned int> > mvBackupInvertedFileId;

   // Mutex
   std::mutex mMutex;

};

} //namespace ORB_SLAM

#endif
