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
    
        //KeyFrame firstKeyFrame;
        //firstKeyFrame.id = 0;
        //mMappoint->pushKeyFrame(firstKeyFrame);
    }

}


void Mappoint::pushRoadsign(Roadsign roadsign)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 独占
    Mappoint::mMappoint->mRoadsigns.push_back(roadsign);
}
void Mappoint::remainRoadsign(uint remain_num)
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

void Mappoint::readOneRoadsign(uint id, Roadsign & roadsign)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 共享
    roadsign.id = 0;
    for (uint i = Mappoint::mMappoint->mRoadsigns.size(); i > 0; i--) {
        if (Mappoint::mMappoint->mRoadsigns[i-1].id == id) {
            Roadsign road(Mappoint::mMappoint->mRoadsigns[i-1]);
            roadsign = road;
        }
    }
}

void Mappoint::refreshRoadsign(std::vector<Roadsign> & roadsigns)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_roadsign); // 独占
    Mappoint::mMappoint->mRoadsigns.swap(roadsigns);

}


void Mappoint::pushComFrame(ComFrame comframe)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_comframe); // 独占
    Mappoint::mMappoint->mComFrame.push_back(comframe);
}
void Mappoint::remainComFrame(uint remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_comframe); // 独占
    if (Mappoint::mMappoint->mComFrame.size() <= remain_num) {
        return ;
    }
    uint delete_num = Mappoint::mMappoint->mComFrame.size() - remain_num;
    Mappoint::mMappoint->mComFrame.erase(Mappoint::mMappoint->mComFrame.begin(), 
                            Mappoint::mMappoint->mComFrame.begin() + delete_num);
}
void Mappoint::readComFrame(std::vector<ComFrame> & comframes)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_comframe); // 共享
    std::vector<ComFrame> temp(Mappoint::mMappoint->mComFrame);
    comframes.swap(temp);

}
void Mappoint::readOneComFrame(uint id, ComFrame & comframe)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_comframe); // 共享
    comframe.id = 0;
    comframe.image.release();
    comframe.ptr.clear();
    comframe.ptr_status.clear();
    for (uint i = 0; i < Mappoint::mMappoint->mComFrame.size(); i++) {
        if (Mappoint::mMappoint->mComFrame[i].id == id) {
            ComFrame f(Mappoint::mMappoint->mComFrame[i]);
            comframe = f;
        }
    }
}
void Mappoint::refreshComFrame(std::vector<ComFrame> & comframes)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_comframe); // 独占
    Mappoint::mMappoint->mComFrame.swap(comframes);

}



void Mappoint::pushKeyFrame(KeyFrame keyframe)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 独占
    Mappoint::mMappoint->mKeyframe.push_back(keyframe);
}
void Mappoint::remainKeyFrame(uint remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 独占
    if (Mappoint::mMappoint->mKeyframe.size() <= remain_num) {
        return ;
    }
    uint delete_num = Mappoint::mMappoint->mKeyframe.size() - remain_num;
    Mappoint::mMappoint->mKeyframe.erase(Mappoint::mMappoint->mKeyframe.begin(), 
                               Mappoint::mMappoint->mKeyframe.begin() + delete_num);
}
void Mappoint::readKeyFrame(std::vector<KeyFrame> & keyframes)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 共享
    std::vector<KeyFrame> temp(Mappoint::mMappoint->mKeyframe);
    keyframes.swap(temp);
}
void Mappoint::readLastKeyFrame(KeyFrame & keyframe)
{
    boost::shared_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 共享
    uint l = Mappoint::mMappoint->mKeyframe.size();
    if (l < 1) {
        keyframe.id = 0;
    } else {
        KeyFrame temp(Mappoint::mMappoint->mKeyframe[l - 1]);
        keyframe = temp;
    }
}
void Mappoint::refreshKeyFrame(std::vector<KeyFrame> & keyframes)
{
    boost::unique_lock<boost::shared_mutex> m(Mappoint::mMappoint->shr_mutex_keyframe); // 独占
    Mappoint::mMappoint->mKeyframe.swap(keyframes);
}



  


}