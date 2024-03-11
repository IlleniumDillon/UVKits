/*
 * @FilePath: gridMap.cc
 * @Author: Ballade-F     258300018@qq.com
 * @Date: 2024-02-03 10:58:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-02-08 11:07:28
 * Copyright: 2024  All Rights Reserved.
 * @Descripttion: 
 */
#include "gridMap.hpp"

void GridMap::initMap(ros::NodeHandle &nh)
{
//TODO:1.获取参数

//TODO:2.ROS话题、定时器相关

//TODO:3.各种flag初始化置位
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(md_.depth_image_);

    // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

    /* get pose */
    md_.camera_pos_(0) = pose->pose.position.x;
    md_.camera_pos_(1) = pose->pose.position.y;
    md_.camera_pos_(2) = pose->pose.position.z;
    md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                         pose->pose.orientation.y, pose->pose.orientation.z)
                          .toRotationMatrix();
    if (isInMap(md_.camera_pos_))
    {
        md_.has_odom_ = true;
        md_.update_num_ += 1;
        md_.occ_need_update_ = true;
    }
    else
    {
        md_.occ_need_update_ = false;
    }

    md_.flag_use_depth_fusion = true;
}


void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
    if (md_.last_occ_update_time_.toSec() < 1.0)
        md_.last_occ_update_time_ = ros::Time::now();

    if (!md_.occ_need_update_)
    {
        if (md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
        {
            ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                      ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
            md_.flag_depth_odom_timeout_ = true;
        }
        return;
    }
    md_.last_occ_update_time_ = ros::Time::now();

    /* update occupancy */
    // ros::Time t1, t2, t3, t4;
    // t1 = ros::Time::now();

    projectDepthImage();
    // t2 = ros::Time::now();
    raycastProcess();
    // t3 = ros::Time::now();

    if (md_.local_updated_)
        clearAndInflateLocalMap();

    // t4 = ros::Time::now();

    // cout << setprecision(7);
    // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

    // md_.fuse_time_ += (t2 - t1).toSec();
    // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

    // if (mp_.show_occ_time_)
    //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
    //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
}

void GridMap::projectDepthImage()
{
    // md_.proj_points_.clear();
    md_.proj_points_cnt = 0;

    uint16_t *row_ptr;
    // int cols = current_img_.cols, rows = current_img_.rows;
    int cols = md_.depth_image_.cols;
    int rows = md_.depth_image_.rows;
    int skip_pix = mp_.skip_pixel_;

    double depth;

    Eigen::Matrix3d camera_r = md_.camera_r_m_;

    /* use depth filter */

        if (!md_.has_first_depth_)
            md_.has_first_depth_ = true;
        else
        {
            Eigen::Vector3d pt_cur, pt_world, pt_reproj;

            Eigen::Matrix3d last_camera_r_inv;
            last_camera_r_inv = md_.last_camera_r_m_.inverse();
            const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

            for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
            {
                row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

                for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
                     u += mp_.skip_pixel_)
                {
                    depth = (*row_ptr) * inv_factor;
                    row_ptr = row_ptr + mp_.skip_pixel_;
//TODO: 测试深度图像素值意义，以及太近太远会出现什么值
                    // project to world frame
                    pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
                    pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
                    pt_cur(2) = depth;

                    pt_world = camera_r * pt_cur + md_.camera_pos_;
                    md_.proj_points_[md_.proj_points_cnt++] = pt_world;

                }
            }
        }

    /* maintain camera pose for consistency check */

    md_.last_camera_pos_ = md_.camera_pos_;
    md_.last_camera_r_m_ = md_.camera_r_m_;
    md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
    // if (md_.proj_points_.size() == 0)
    if (md_.proj_points_cnt == 0)
        return;

    ros::Time t1, t2;

    md_.raycast_num_ += 1;

    int vox_idx;
    double length;

    // bounding box of updated region
    double min_x = mp_.map_max_boundary_(0);
    double min_y = mp_.map_max_boundary_(1);
    double min_z = mp_.map_max_boundary_(2);

    double max_x = mp_.map_min_boundary_(0);
    double max_y = mp_.map_min_boundary_(1);
    double max_z = mp_.map_min_boundary_(2);

    RayCaster raycaster;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt, pt_w;

    for (int i = 0; i < md_.proj_points_cnt; ++i)
    {
        pt_w = md_.proj_points_[i];

        // set flag for projected point

        if (!isInMap(pt_w))
        {
            pt_w = closetPointInMap(pt_w, md_.camera_pos_);

            length = (pt_w - md_.camera_pos_).norm();
            if (length > mp_.max_ray_length_)
            {
                pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
            }
            vox_idx = setCacheOccupancy(pt_w, 0);
        }
        else
        {
            length = (pt_w - md_.camera_pos_).norm();

            if (length > mp_.max_ray_length_)
            {
                pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
                vox_idx = setCacheOccupancy(pt_w, 0);
            }
            else
            {
                vox_idx = setCacheOccupancy(pt_w, 1);
            }
        }

        max_x = max(max_x, pt_w(0));
        max_y = max(max_y, pt_w(1));
        max_z = max(max_z, pt_w(2));

        min_x = min(min_x, pt_w(0));
        min_y = min(min_y, pt_w(1));
        min_z = min(min_z, pt_w(2));

        // raycasting between camera center and point

        if (vox_idx != INVALID_IDX)
        {
            if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
            {
                continue;
            }
            else
            {
                md_.flag_rayend_[vox_idx] = md_.raycast_num_;
            }
        }

        raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

        while (raycaster.step(ray_pt))
        {
            Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
            length = (tmp - md_.camera_pos_).norm();

            // if (length < mp_.min_ray_length_) break;

            vox_idx = setCacheOccupancy(tmp, 0);

            if (vox_idx != INVALID_IDX)
            {
                if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
                {
                    break;
                }
                else
                {
                    md_.flag_traverse_[vox_idx] = md_.raycast_num_;
                }
            }
        }
    }

    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));

    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));
    max_z = max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.local_updated_ = true;

    // update occupancy cached in queue
    Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
    Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

    while (!md_.cache_voxel_.empty())
    {

        Eigen::Vector3i idx = md_.cache_voxel_.front();
        int idx_ctns = toAddress(idx);
        md_.cache_voxel_.pop();

        double log_odds_update =
            md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

        md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

        if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
        {
            continue;
        }
        else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
        {
            md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
            continue;
        }

        bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
        if (!in_local)
        {
            md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
        }

        md_.occupancy_buffer_[idx_ctns] =
            std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                     mp_.clamp_max_log_);
    }
}

void GridMap::clearAndInflateLocalMap()
{
    /*clear outside local*/
    const int vec_margin = 5;
    // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
    // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
    // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

    Eigen::Vector3i min_cut = md_.local_bound_min_ -
                              Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector3i max_cut = md_.local_bound_max_ +
                              Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    boundIndex(min_cut_m);
    boundIndex(max_cut_m);

    // clear data outside the local range

    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
        for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
        {

            for (int z = min_cut_m(2); z < min_cut(2); ++z)
            {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            }

            for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
            {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            }
        }

    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
        for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
        {

            for (int y = min_cut_m(1); y < min_cut(1); ++y)
            {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            }

            for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
            {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            }
        }

    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
        for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
        {

            for (int x = min_cut_m(0); x < min_cut(0); ++x)
            {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            }

            for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
            {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            }
        }

    // inflate occupied voxels to compensate robot size

    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    // int inf_step_z = 1;
    std::vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
    // inf_pts.resize(4 * inf_step + 3);
    Eigen::Vector3i inf_pt;

    // clear outdated data
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
            for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
            {
                md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
            }

    // inflate obstacles
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
            for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
            {

                if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
                {
                    inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

                    for (int k = 0; k < (int)inf_pts.size(); ++k)
                    {
                        inf_pt = inf_pts[k];
                        int idx_inf = toAddress(inf_pt);
                        if (idx_inf < 0 ||
                            idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
                        {
                            continue;
                        }
                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
                    }
                }
            }
}


int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
    if (occ != 1 && occ != 0)
        return INVALID_IDX;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    int idx_ctns = toAddress(id);

    md_.count_hit_and_miss_[idx_ctns] += 1;

    if (md_.count_hit_and_miss_[idx_ctns] == 1)
    {
        md_.cache_voxel_.push(id);
    }

    if (occ == 1)
        md_.count_hit_[idx_ctns] += 1;

    return idx_ctns;
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
    Eigen::Vector3d diff = pt - camera_pt;
    Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
    Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

    double min_t = 1000000;

    for (int i = 0; i < 3; ++i)
    {
        if (fabs(diff[i]) > 0)
        {

            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;

            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }

    return camera_pt + (min_t - 1e-3) * diff;
}