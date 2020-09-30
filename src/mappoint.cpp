/** @file mappoint.cpp
 * @brief 地图
 * 
 * @author Tamray
 * @date 2020.09.23
 */

#include "trslam/mappoint.h"

namespace trslam {

std::shared_ptr<Mappoint> Mappoint::mMappoint = nullptr;

void Mappoint::createMappoint()
{
    if (mMappoint == nullptr) {
        mMappoint = std::shared_ptr<Mappoint>(new Mappoint);
    }
}

void Mappoint::pushPosture(Postrue posture)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_posture); // 独占
    Mappoint::mMappoint->mPostures.push_back(posture);
}

void Mappoint::erasePosture(uint remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_posture); // 独占
    if (Mappoint::mMappoint->mPostures.size() <= remain_num) {
        return ;
    }
    uint delete_num = Mappoint::mMappoint->mPostures.size() - remain_num;
    Mappoint::mMappoint->mPostures.erase(Mappoint::mMappoint->mPostures.begin(), 
                               Mappoint::mMappoint->mPostures.begin() + delete_num);
}

void Mappoint::readPostrue(std::vector<Postrue> & postures)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_posture); // 共享
    std::vector<Postrue> temp(Mappoint::mMappoint->mPostures);
    postures.swap(temp);
}

void Mappoint::refreshPosture(std::vector<Postrue> & postures)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_posture); // 独占
    Mappoint::mMappoint->mPostures.swap(postures);
}





void Mappoint::pushRoadsign(Roadsign roadsign)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 独占
    Mappoint::mMappoint->mRoadsigns.push_back(roadsign);
}
void Mappoint::eraseRoadsign(uint remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 独占
    if (Mappoint::mMappoint->mRoadsigns.size() <= remain_num) {
        return ;
    }
    uint delete_num = Mappoint::mMappoint->mRoadsigns.size() - remain_num;
    Mappoint::mMappoint->mRoadsigns.erase(Mappoint::mMappoint->mRoadsigns.begin(), 
                                mRoadsigns.begin() + delete_num);
}
void Mappoint::readRoadsign(std::vector<Roadsign> & roadsigns)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 共享
    std::vector<Roadsign> temp(Mappoint::mMappoint->mRoadsigns);
    roadsigns.swap(temp);

}
void Mappoint::refreshRoadsign(std::vector<Roadsign> & roadsigns)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 独占
    Mappoint::mMappoint->mRoadsigns.swap(roadsigns);

}




void Mappoint::pushFramefeature(Framefeature feature)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_frame); // 独占
    Mappoint::mMappoint->mFramefeatures.push_back(feature);
}
void Mappoint::eraseFramefeature(uint remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_frame); // 独占
    if (Mappoint::mMappoint->mFramefeatures.size() <= remain_num) {
        return ;
    }
    uint delete_num = Mappoint::mMappoint->mFramefeatures.size() - remain_num;
    Mappoint::mMappoint->mFramefeatures.erase(Mappoint::mMappoint->mFramefeatures.begin(), 
                            Mappoint::mMappoint->mFramefeatures.begin() + delete_num);

}
void Mappoint::readFramefeature(std::vector<Framefeature> & features)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_frame); // 共享
    std::vector<Framefeature> temp(Mappoint::mMappoint->mFramefeatures);
    features.swap(temp);

}
void Mappoint::refreshFramefeature(std::vector<Framefeature> & features)
{

    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_frame); // 独占
    Mappoint::mMappoint->mFramefeatures.swap(features);
}




void Mappoint::pushKeyframe(uint id)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 独占
    Mappoint::mMappoint->mKeyframe.push_back(id);
}
void Mappoint::eraseKeyframe(uint remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 独占
    if (Mappoint::mMappoint->mKeyframe.size() <= remain_num) {
        return ;
    }
    uint delete_num = Mappoint::mMappoint->mKeyframe.size() - remain_num;
    Mappoint::mMappoint->mKeyframe.erase(Mappoint::mMappoint->mKeyframe.begin(), 
                               Mappoint::mMappoint->mKeyframe.begin() + delete_num);

}
void Mappoint::readKeyframe(std::vector<uint> & keyframes)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 共享
    std::vector<uint> temp(Mappoint::mMappoint->mKeyframe);
    keyframes.swap(temp);

}
void Mappoint::refreshKeyframe(std::vector<uint> & keyframes)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 独占
    Mappoint::mMappoint->mKeyframe.swap(keyframes);
}



  


}