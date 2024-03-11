#ifndef _GRIDMAP_HPP_
#define _GRIDMAP_HPP_

#include <iostream>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <vector>
#include <queue>
#include <string>
#include "raycast.h"

struct MappingParameters
{

    /* map properties */
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
    Eigen::Vector3i map_voxel_num_;                       // map range in index
    Eigen::Vector3d local_update_range_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;

    /* camera parameters */
    double cx_, cy_, fx_, fy_;

    /* depth image projection  */
    int depth_filter_margin_;
    double k_depth_scaling_factor_;
    int skip_pixel_;

    /* raycasting */
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
        min_occupancy_log_;                  // logit of occupancy probability
    double min_ray_length_, max_ray_length_; // range of doing raycasting

    /* local map update and clear */
    int local_map_margin_;

    /* active mapping */
    double unknown_flag_;
};

struct MappingData
{
    // main map data, occupancy of each voxel and Euclidean distance
    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_inflate_;

    // camera position and pose data
    Eigen::Vector3d camera_pos_, last_camera_pos_;
    Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
    Eigen::Matrix4d cam2body_;

    // depth image data
    cv::Mat depth_image_, last_depth_image_;
    int image_cnt_;

    // flags of map state
    bool occ_need_update_, local_updated_;
    bool has_first_depth_;

    // depth image projected point cloud
    std::vector<Eigen::Vector3d> proj_points_;
    int proj_points_cnt;

    // flag buffers for speeding up raycasting
    std::vector<short> count_hit_, count_hit_and_miss_;
    std::vector<char> flag_traverse_, flag_rayend_;
    char raycast_num_;
    std::queue<Eigen::Vector3i> cache_voxel_;

    // range of updating grid
    Eigen::Vector3i local_bound_min_, local_bound_max_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
    GridMap() {}
    ~GridMap() {}

    enum
    {
        INVALID_IDX = -10000
    };

    void initMap(ros::NodeHandle &nh);

    inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
    inline int toAddress(const Eigen::Vector3i &id);
    inline int GridMap::toAddress(int &x, int &y, int &z);
    inline bool GridMap::isInMap(const Eigen::Vector3d &pos);
    inline void GridMap::boundIndex(Eigen::Vector3i &id);
    inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);

    typedef std::shared_ptr<GridMap> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    MappingParameters mp_;
    MappingData md_;

    void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                           const geometry_msgs::PoseStampedConstPtr &pose);

    //定时器中断
    void updateOccupancyCallback(const ros::TimerEvent & /*event*/);

    // main update process
    void projectDepthImage();
    void raycastProcess();
    void clearAndInflateLocalMap();

    int setCacheOccupancy(Eigen::Vector3d pos, int occ);
    Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);


};

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int &x, int &y, int &z)
{
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4)
    {
        // cout << "less than min range!" << endl;
        return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary_(2) - 1e-4)
    {
        return false;
    }
    return true;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z)
            {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
            }
}

#endif