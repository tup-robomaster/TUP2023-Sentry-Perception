#include "../../include/armor_detector/armor_detector.hpp"

using namespace std;
namespace armor_detector
{
    Detector::Detector(const PathParam& path_param, const DetectorParam& detector_params, const DebugParam& debug_params) 
    : detector_params_(detector_params), 
    path_params_(path_param), debug_params_(debug_params), logger_(rclcpp::get_logger("armor_detector"))
    {
        //初始化
        lost_cnt = 0;
        is_last_target_exists = false;
        last_target_area = 0;
        input_size = {640, 480};
        is_init_ = false;

    }

    Detector::~Detector()
    {

    }

    /**
     * @brief 车辆装甲板检测
     * 
     * @param src 图像数据结构体
     * @param is_target_lost 目标是否丢失
     * @return true 
     * @return false 
     */
    bool Detector::armor_detect(TaskData &src, bool& is_target_lost, std::vector<Armor> &last_armors)
    {

        
        time_start = steady_clock_.now();

        auto input = src.img;
        timestamp = src.timestamp;
       

        // Eigen::Matrix3d rmat_imu;
       
        rmat_imu = Eigen::Matrix3d::Identity();

        time_crop = steady_clock_.now();
        last_armors.clear();
        objects.clear();
        armors.clear();
        
        if(!armor_detector_.detect(input, objects))
        {   //若未检测到目标

            is_target_lost = true;
            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0.0;
            return false;
        }

        time_infer = steady_clock_.now();
        
        //将对象排序，保留面积较大的对象
        sort(objects.begin(),objects.end(),[](ArmorObject& prev, ArmorObject& next)
        {
            return prev.area > next.area;
        });

        //若对象较多保留前按面积排序后的前max_armors个
        if ((int)(objects.size()) > this->detector_params_.max_armors_cnt)
            objects.resize(this->detector_params_.max_armors_cnt);
        
        //生成装甲板对象
        for (auto object : objects)
        {
            if(detector_params_.color == RED)
            {
                if (object.color != 1)
                    continue;
            }
            else
            {
                if (object.color != 0)
                    continue;
            }
   
            Armor armor;
            armor.id = object.cls;
            // RCLCPP_INFO(logger_, "armor id:%d",armor.id );
            armor.color = object.color;
            armor.conf = object.prob;
            if (object.color == 0)
                armor.key = "B" + to_string(object.cls);
            if (object.color == 1)
                armor.key = "R" + to_string(object.cls);
            if (object.color == 2)
                armor.key = "N" + to_string(object.cls);
            if (object.color == 3)
                armor.key = "P" + to_string(object.cls);
            memcpy(armor.apex2d, object.apex, 4 * sizeof(cv::Point2f));
            for(int i = 0; i < 4; i++)
                armor.apex2d[i] += Point2f((float)roi_offset.x,(float)roi_offset.y);
            Point2f apex_sum;
            for(auto apex : armor.apex2d)
                apex_sum +=apex;
            armor.center2d = apex_sum / 4.f;

            //生成装甲板旋转矩形和ROI
            std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
            RotatedRect points_pic_rrect = minAreaRect(points_pic);        
            armor.rrect = points_pic_rrect;
            auto bbox = points_pic_rrect.boundingRect();
            auto x = bbox.x - 0.5 * bbox.width * (detector_params_.armor_roi_expand_ratio_width - 1);
            auto y = bbox.y - 0.5 * bbox.height * (detector_params_.armor_roi_expand_ratio_height - 1);
            armor.roi = Rect(x,
                            y,
                            bbox.width * detector_params_.armor_roi_expand_ratio_width,
                            bbox.height * detector_params_.armor_roi_expand_ratio_height
                            );
            //若装甲板置信度小于高阈值，需要相同位置存在过装甲板才放行
            if (armor.conf < this->detector_params_.armor_conf_high_thres)
            {
                if (last_armors.empty())
                {
                    continue;
                }
                else
                {
                    bool is_this_armor_available = false;
                    for (auto last_armor : last_armors)
                    {
                        if (last_armor.roi.contains(armor.center2d))
                        {
                            is_this_armor_available = true;
                            break;
                        }
                    }
                    if (!is_this_armor_available)
                    {
                        continue;
                        cout << "IGN" << endl;
                    }
                }
            }
            //进行PnP，目标较少时采取迭代法，较多时采用IPPE
            int pnp_method;
            if (objects.size() <= 2)
                pnp_method = SOLVEPNP_ITERATIVE;
            else
                pnp_method = SOLVEPNP_IPPE;
            TargetType target_type = SMALL;

            //计算长宽比,确定装甲板类型
            auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
                                    min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            //若大于长宽阈值或为哨兵、英雄装甲板
            if (object.cls == 1 || object.cls == 0)
                target_type = BIG;
            //FIXME：若存在平衡步兵需要对此处步兵装甲板类型进行修改
            else if (object.cls == 2 || object.cls == 3 || object.cls == 4 || object.cls == 5 || object.cls == 6)
                target_type = SMALL;
            else if(apex_wh_ratio > detector_params_.armor_type_wh_thres)
                target_type = BIG;

            //单目PnP
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_ITERATIVE);
            // auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
            
            //防止装甲板类型出错导致解算问题，首先尝试切换装甲板类型，若仍无效则直接跳过该装甲板
            if (pnp_result.armor_cam.norm() > 10 ||
                isnan(pnp_result.armor_cam[0]) ||
                isnan(pnp_result.armor_cam[1]) ||
                isnan(pnp_result.armor_cam[2]))
            {
                if (target_type == SMALL)
                    target_type = BIG;
                else if (target_type == BIG)
                    target_type = SMALL;
                pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
                if (pnp_result.armor_cam.norm() > 10 ||
                    isnan(pnp_result.armor_cam[0]) ||
                    isnan(pnp_result.armor_cam[1]) ||
                    isnan(pnp_result.armor_cam[2]))
                    {
                        continue;
                    }
            }

            armor.armor3d_world = pnp_result.armor_world;
            armor.armor3d_cam = pnp_result.armor_cam;
            armor.euler = pnp_result.euler;
            armor.rmat = pnp_result.rmat;
            armor.area = object.area;
            armors.push_back(armor);
        }
        
        //若无合适装甲板
        if (armors.empty())
        {
            RCLCPP_WARN_THROTTLE(logger_, this->steady_clock_, 500, "No suitable targets...");
            // if(debug_params_.show_aim_cross)
            // {
                // line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
                // line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
            // }

            // if(debug_params_.show_all_armors)
            // {
            //     RCLCPP_DEBUG_ONCE(logger_, "Show all armors...");
            //     // RCLCPP_INFO(logger_, "=========================");
            //     showArmors(src);
            // }

            is_target_lost = true;
            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0;
            return false;
        }
        else
        {
            last_armors.clear();
            last_armors = armors;
            for(int i = 0; i < armors.size(); i++)
            {
            RCLCPP_INFO(logger_, "armor id:%d",last_armors[i].id );
            }
            // robot_detect(src);
            // cout<<"final_robot:"<<final_robot_results.size()<<endl;
           if(debug_params_.show_all_armors)
            {
                RCLCPP_DEBUG_ONCE(logger_, "Show all armors...");
                // RCLCPP_INFO(logger_, "=========================");
                showArmors(src);
            } 
            
        }
        is_target_lost = false;
        return true;
    }

 
    /**
     * @brief 显示检测到的装甲板信息
     * 
     * @param src 图像数据结构体
     */
    void Detector::showArmors(TaskData& src)
    {
        // RCLCPP_INFO(logger_, "=========================");
        for (auto armor : armors)
        {
            // RCLCPP_INFO(logger_, "=========================");
            char ch[10];
            sprintf(ch, "%.3f", armor.conf);
            std::string conf_str = ch;
            putText(src.img, conf_str, armor.apex2d[3], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

            std::string id_str = to_string(armor.id);
            if (armor.color == 0)
                putText(src.img, "B" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
            if (armor.color == 1)
                putText(src.img, "R" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
            if (armor.color == 2)
                putText(src.img, "N" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
            if (armor.color == 3)
                putText(src.img, "P" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 255}, 2);
            for(int i = 0; i < 4; i++)
            {
                line(src.img, armor.apex2d[i % 4], armor.apex2d[(i + 1) % 4], {0,255,0}, 1);
                // cout << "armor3d_world: " << armor.armor3d_world << endl;
                // cout << "armor3d_cam: " << armor.armor3d_cam << endl;
                // cout << "rect: " << armor.rect << endl;
                // cout << "armor.apex2d: " << armor.apex2d << endl;
                // cout << "armor.roi: " << armor.roi << endl;
                // cout << "armor.type: " << armor.type<< endl;


            }
            rectangle(src.img, armor.roi, {255, 0, 255}, 1);
            // auto armor_center = coordsolver_.reproject(armor.armor3d_cam);
            // circle(src.img, armor_center, 4, {0, 0, 255}, 2);
        }
        // for (auto car : final_robot_results)
        // {
        //     // RCLCPP_INFO(logger_, "=========================");
        //     rectangle(src.img, car.m_rect, {255, 0, 255}, 1);
        // }
    }
    // void Detector::showCar(TaskData& src)
    // {
    //     // RCLCPP_INFO(logger_, "=========================");
    //     for (auto car : final_robot_results)
    //     {
    //         // RCLCPP_INFO(logger_, "=========================");
    //         rectangle(src.img, car.m_rect, {255, 0, 255}, 1);
    //     }
    // }
    // void load_camera_params(const std::vector<std::vector<double>> intrinsics,
    //                                        const std::vector<std::vector<double>> extrinsics,
    //                                        const std::vector<std::vector<double>> discoeffs)
    // {
    //     for (int i = 0; i < intrinsics.size(); i++)
    //     {
    //         cv::Mat intrinsic_temp = cv::Mat(3, 3, CV_64FC1);
    //         memcpy(intrinsic_temp.data, intrinsics[i].data(), intrinsics[i].size() * sizeof(double));
    //         _intrinsic_cvs.push_back(intrinsic_temp);
    //     }
    //     for (int i = 0; i < extrinsics.size(); i++)
    //     {
    //         cv::Mat extrinsic_temp = cv::Mat(4, 4, CV_64FC1);
    //         memcpy(extrinsic_temp.data, extrinsics[i].data(), extrinsics[i].size() * sizeof(double));
    //         _extrinsic_cvs.push_back(extrinsic_temp);
    //     }
    //     for (int i = 0; i < discoeffs.size(); i++)
    //     {
    //         _discoff_cvs.push_back(cv::Mat(discoeffs[i]).t());
    //     }
    // }
    // void location_estimation(const int id, std::vector<Robot> &Robots, bool is_robot_from_armors)
    // {

    //     for (int i = 0; i < Robots.size(); ++i)
    //     {
    //         float fx = _intrinsic_cvs[id].at<double>(0, 0);
    //         float fy = _intrinsic_cvs[id].at<double>(1, 1);
    //         float cx = _intrinsic_cvs[id].at<double>(0, 2);
    //         float cy = _intrinsic_cvs[id].at<double>(1, 2);
    //         float center_u = Robots[i].m_rect.x + Robots[i].m_rect.width / 2;
    //         float center_v = Robots[i].m_rect.y + Robots[i].m_rect.height / 2;

    //         if (!is_robot_from_armors)
    //         {
    //             Robots[i].XYZ_camera.z;
    //         }
    //         else
    //         {
    //             Robots[i].XYZ_camera.z = 0.35;
    //         }
    //         Robots[i].XYZ_camera.x = (center_u - cx) * Robots[i].XYZ_camera.z / fx;
    //         Robots[i].XYZ_camera.y = (center_v - cy) * Robots[i].XYZ_camera.z / fy;

    //         Eigen::Matrix4f extrinsic_temp;
    //         cv::cv2eigen(_extrinsic_cvs[id], extrinsic_temp);
    //         Eigen::Vector3f P_camera;
    //         P_camera << Robots[i].XYZ_camera.x, Robots[i].XYZ_camera.y, Robots[i].XYZ_camera.z;
    //         Eigen::Matrix3f Rbc = extrinsic_temp.topLeftCorner(3, 3).inverse();
    //         Eigen::Vector3f tbc = -Rbc * extrinsic_temp.topRightCorner(3, 1);
    //         Eigen::Vector3f P_body = Rbc * P_camera + tbc;
    //         Robots[i].XYZ_world.x = P_body[0];
    //         Robots[i].XYZ_world.y = P_body[1];
    //         Robots[i].XYZ_world.z = 0;
    //         //std::cout<<"In Camera "<< id << " Find "<< "Robot "<< Robots[i].m_id <<" x: "<< Robots[i].XYZ_world.x << " y: "<< Robots[i].XYZ_world.y << " z: "<< Robots[i].XYZ_world.z <<std::endl;
    //     }
    // }


//     void Detector::RobotMatch(const int id, const std::vector<Armor> &results, std::vector<Robot> &Robots)
//     {
//         std::vector<Armor> last;
//         // std::vector<Robot> Robots;
//         bool flag = false;
//         int numarmor = 0;
//         std::vector<int> repeat_index;
//         for (int i = 0; i < results.size(); i++)
//         {
//             // RCLCPP_INFO(logger_, "111111111111111111111111");
//             for(int j = i + 1; j < results.size(); j++)
//             {
//                 // RCLCPP_INFO(logger_, "2222222222222222222222222");
//                 if(results[i].id == results[j].id)
//                 {
//                     float dis = sqrt(pow((results[i].armor3d_world[0] - results[j].armor3d_world[0]),2)+
//                             pow((results[i].armor3d_world[1] - results[j].armor3d_world[1]),2));
//                     if(dis < 0.5)
//                     {
//                         // RCLCPP_INFO(logger_, "333333333333333333333333333");
//                         flag = true;
//                         repeat_index.push_back(i);
//                         // Robot robot_temp = results[j];
                        
//                         int k = numarmor;
//                         Robots[k].m_id = results[i].id;
//                         Robots[k].m_color = results[i].color;
//                         Robots[k].m_rect.x = results[i].roi.x;
//                         Robots[k].m_rect.y = results[i].roi.y;
//                         Robots[k].XYZ_world[0] = (results[i].armor3d_world[0] + results[j].armor3d_world[1])/2;
//                         Robots[k].XYZ_world[1] = (results[i].armor3d_world[1] + results[j].armor3d_world[1])/2;
//                         Robots[k].m_rect.x = min(results[i].roi.x, results[j].roi.x);
//                         Robots[k].m_rect.y = min(results[i].roi.y, results[j].roi.y);
//                         Robots[k].m_number = min(results[i].conf, results[j].conf);
//                         if(results[i].roi.x <= results[j].roi.x)
//                             Robots[k].m_rect.width = results[j].roi.x + results[j].roi.width - results[i].roi.x ;
//                         else
//                             Robots[k].m_rect.width = results[i].roi.x + results[i].roi.width - results[j].roi.x ;
//                         if(results[i].roi.y <= results[j].roi.y)
//                             Robots[k].m_rect.height = results[j].roi.y + results[j].roi.height - results[i].roi.y ;
//                         else
//                             Robots[k].m_rect.height = results[i].roi.y + results[i].roi.height - results[j].roi.y ;

//                         numarmor++;
//                         robot_results.push_back(Robots[k]);
//                         continue;
//                     }
//                 }
//             }
//         }
//     }
//     void Detector::robot_detect(TaskData& src)
//     {
//         std::vector<std::vector<Armor>> detected_objects;
//         // std::vector<Robot> robot_result;
//         // std::vector<Robot> final_robot_results;
//         detected_objects.clear();
//         detected_objects.push_back(armors);
//     //    if(sync_packages(Measures, detected_objects))
//     //    {
//             for(int i = 0 ; i < detected_objects.size(); i++)
//             {
//                 // RCLCPP_INFO(this->get_logger(), "---------------------------");
                
//                 std::vector<Armor> obj = detected_objects[i];
//                 std::vector<Robot> robots_temp;
//                 RobotMatch(i,obj,robots_temp); 
//                 for(int j = 0; j < robot_results.size();j++)
//                 {
//                     if(robot_results[j].m_id != 0)
//                     {
//                         final_robot_results.push_back(robot_results[j]);
//                     }
//                 }
//             }
//         if (final_robot_results.empty())
//         {
//             RCLCPP_WARN_THROTTLE(logger_, this->steady_clock_, 500, "No detect car ...");
//         }
//         else
//         {
//             final_robot_results.clear();
//             final_robot_results = robot_results;

//         }
//     }
}
