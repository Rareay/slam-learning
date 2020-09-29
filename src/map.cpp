/** @file map.cpp
 * @brief 地图
 * 
 * @author Tamray
 * @date 2020.09.23
 */

#include "trslam/map.h"


namespace trslam {

std::shared_ptr<Map> Map::mMap = nullptr;

void Map::createMap()
{
    if (mMap == nullptr) {
        mMap = std::shared_ptr<Map>(new Map);
    }
}

void Map::pushPosture(Postrue posture)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_posture); // 独占
    Map::mMap->mPostures.push_back(posture);
}

void Map::erasePosture(int remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_posture); // 独占
    if (Map::mMap->mPostures.size() <= remain_num) {
        return ;
    }
    unsigned int delete_num = Map::mMap->mPostures.size() - remain_num;
    Map::mMap->mPostures.erase(Map::mMap->mPostures.begin(), 
                               Map::mMap->mPostures.begin() + delete_num);
}

void Map::readPostrue(std::vector<Postrue> & postures)
{
    boost::shared_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_posture); // 共享
    std::vector<Postrue> temp(Map::mMap->mPostures);
    postures.swap(temp);
}

void Map::refreshPosture(std::vector<Postrue> & postures)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_posture); // 独占
    Map::mMap->mPostures.swap(postures);
}





void Map::pushRoadsign(Roadsign roadsign)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_roadsign); // 独占
    Map::mMap->mRoadsigns.push_back(roadsign);
}
void Map::eraseRoadsign(int remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_roadsign); // 独占
    if (Map::mMap->mRoadsigns.size() <= remain_num) {
        return ;
    }
    unsigned int delete_num = Map::mMap->mRoadsigns.size() - remain_num;
    Map::mMap->mRoadsigns.erase(Map::mMap->mRoadsigns.begin(), 
                                mRoadsigns.begin() + delete_num);
}
void Map::readRoadsign(std::vector<Roadsign> & roadsigns)
{
    boost::shared_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_roadsign); // 共享
    std::vector<Roadsign> temp(Map::mMap->mRoadsigns);
    roadsigns.swap(temp);

}
void Map::refreshRoadsign(std::vector<Roadsign> & roadsigns)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_roadsign); // 独占
    Map::mMap->mRoadsigns.swap(roadsigns);

}




void Map::pushFramefeature(Framefeature feature)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_frame); // 独占
    Map::mMap->mFramefeatures.push_back(feature);
}
void Map::eraseFramefeature(int remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_frame); // 独占
    if (Map::mMap->mFramefeatures.size() <= remain_num) {
        return ;
    }
    unsigned int delete_num = Map::mMap->mFramefeatures.size() - remain_num;
    Map::mMap->mFramefeatures.erase(Map::mMap->mFramefeatures.begin(), 
                            Map::mMap->mFramefeatures.begin() + delete_num);

}
void Map::readFramefeature(std::vector<Framefeature> & features)
{
    boost::shared_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_frame); // 共享
    std::vector<Framefeature> temp(Map::mMap->mFramefeatures);
    features.swap(temp);

}
void Map::refreshFramefeature(std::vector<Framefeature> & features)
{

    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_frame); // 独占
    Map::mMap->mFramefeatures.swap(features);
}




void Map::pushKeyframe(unsigned int id)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_keyframe); // 独占
    Map::mMap->mKeyframe.push_back(id);
}
void Map::eraseKeyframe(int remain_num)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_keyframe); // 独占
    if (Map::mMap->mKeyframe.size() <= remain_num) {
        return ;
    }
    unsigned int delete_num = Map::mMap->mKeyframe.size() - remain_num;
    Map::mMap->mKeyframe.erase(Map::mMap->mKeyframe.begin(), 
                               Map::mMap->mKeyframe.begin() + delete_num);

}
void Map::readKeyframe(std::vector<unsigned int> & keyframes)
{
    boost::shared_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_keyframe); // 共享
    std::vector<unsigned int> temp(Map::mMap->mKeyframe);
    keyframes.swap(temp);

}
void Map::refreshKeyframe(std::vector<unsigned int> & keyframes)
{
    boost::unique_lock<boost::shared_mutex> m(Map::mMap->shr_mutex_keyframe); // 独占
    Map::mMap->mKeyframe.swap(keyframes);
}



    
Eigen::Vector4d Map::readParam()
{
    return Map::mMap->mParam;
}




}