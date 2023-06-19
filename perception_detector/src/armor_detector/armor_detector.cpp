#include "../../include/armor_detector/armor_detector.hpp"

using namespace std;



namespace perception_detector
{
    inline Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
    {
        /**
         * @brief transform rotatedMatrix to euler angle
         * 
         */
        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;
        
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }

        return {z, y, x};
    }

    Detector::Detector(const PathParam& path_param, const DetectorParam& detector_params, const DebugParam& debug_params) 
    : detector_params_(detector_params), 
    path_params_(path_param), debug_params_(debug_params), logger_(rclcpp::get_logger("armor_detector"))
    {
        //初始化
        lost_cnt = 0;
        is_last_target_exists = false;
        last_target_area = 0;
        input_size = {640, 480};
        armor_detector_.initModel(path_param.network_path);

    }

    Detector::~Detector()
    {
    }


    /**
     * @brief 车辆装甲板检测
     * 
     * @param src 图像数据
     * @return true 
     * @return false 
     */
    bool Detector::detect(cv::Mat &src, std::vector<Armor> &armors)
    {
        time_start = steady_clock_.now();
        auto input = src;
        time_crop = steady_clock_.now();
        objects.clear();
        
        if(!armor_detector_.detect(input, objects))
        {   //若未检测到目标
            lost_cnt++;
            is_last_target_exists = false;
            last_armors.clear();
            last_target_area = 0.0;
            return false;
        }
        time_infer = steady_clock_.now();
        //生成装甲板对象
        for (auto object : objects)
        {
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
            TargetType target_type = SMALL;

            //计算长宽比,确定装甲板类型
            auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
                                    min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            //若大于长宽阈值或为哨兵、英雄装甲板
            if (object.cls == 1)
                target_type = BIG;
            //FIXME：若存在平衡步兵需要对此处步兵装甲板类型进行修改
            else if (object.cls == 0 || object.cls == 2 || object.cls == 3 ||
                         object.cls == 4 || object.cls == 5 || object.cls == 6)
                target_type = SMALL;
            else if(apex_wh_ratio > detector_params_.armor_type_wh_thres)
                target_type = BIG;
            
            //----------------------PnP---------------------------
            std::vector<cv::Point3d> points_world;
            //大于长宽比阈值使用大装甲板世界坐标
            if (target_type == BIG)
            {
                points_world = 
                {
                    {-0.1125, 0.027, 0},
                    {-0.1125, -0.027, 0},
                    {0.1125, -0.027, 0},
                    {0.1125, 0.027, 0}
                };
            }
            else if (target_type == SMALL)
            {
                points_world = 
                {
                    {-0.066, 0.027, 0},
                    {-0.066, -0.027, 0},
                    {0.066, -0.027, 0},
                    {0.066, 0.027, 0}
                };
            }
            cv::Mat rvec;
            cv::Mat rmat;
            cv::Mat tvec;
            Eigen::Matrix3d rmat_eigen;
            Eigen::Vector3d tvec_eigen;
            Eigen::Vector3d coord_camera;

            solvePnP(points_world, points_pic, intrinsic, dis_coeff, rvec, tvec, false, SOLVEPNP_IPPE);
                
            //Pc = R * Pw + T
            Rodrigues(rvec, rmat);
            cv2eigen(rmat, rmat_eigen);
            cv2eigen(tvec, tvec_eigen);

            //防止装甲板类型出错导致解算问题，首先尝试切换装甲板类型，若仍无效则直接跳过该装甲板
            if (tvec_eigen.norm() > 8 ||
                isnan(tvec_eigen[0]) ||
                isnan(tvec_eigen[1]) ||
                isnan(tvec_eigen[2]))
                    continue;
            armor.armor3d_cam = tvec_eigen;
            armor.euler = rotationMatrixToEulerAngles(rmat_eigen);
            armor.rmat = rmat_eigen;
            armor.area = object.area;
            armors.push_back(armor);
        }
        //若无合适装甲板
        if (armors.empty())
        {
            RCLCPP_WARN_THROTTLE(logger_, this->steady_clock_, 500, "No suitable targets...");
            last_armors.clear();
            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0;
            return false;
        }
        else
        {
            last_armors = armors;
           if(debug_params_.show_all_armors)
            {
                RCLCPP_DEBUG_ONCE(logger_, "Show all armors...");
                showArmors(src,armors);
            } 
        }
        
        return true;
    }

 
    /**
     * @brief 显示检测到的装甲板信息
     * 
     * @param src 图像数据结构体
     */
    void Detector::showArmors(cv::Mat &src, std::vector<Armor> armors)
    {
        // RCLCPP_INFO(logger_, "=========================");
        for (auto armor : armors)
        {
            // RCLCPP_INFO(logger_, "=========================");
            char ch[10];
            sprintf(ch, "%.3f", armor.conf);
            std::string conf_str = ch;
            putText(src, conf_str, armor.apex2d[3], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

            std::string id_str = to_string(armor.id);
            if (armor.color == 0)
                putText(src, "B" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
            if (armor.color == 1)
                putText(src, "R" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
            if (armor.color == 2)
                putText(src, "N" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
            if (armor.color == 3)
                putText(src, "P" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 255}, 2);
            for(int i = 0; i < 4; i++)
                line(src, armor.apex2d[i % 4], armor.apex2d[(i + 1) % 4], {0,255,0}, 1);
            rectangle(src, armor.roi, {255, 0, 255}, 1);
        }
    }

    bool Detector::setCameraIntrinsicsByYAML(const std::string& yaml_file_path)
    {
        // Load camera intrinsics from YAML file
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        // Parse camera matrix from YAML
        std::vector<double> camera_matrix_data = config["camera_matrix"]["data"].as<std::vector<double>>();
        Eigen::Matrix<double, 3, 3> camera_matrix(camera_matrix_data.data());
        camera_matrix.transposeInPlace();

        // Parse distortion coefficients from YAML
        std::vector<double> distortion_coefficients_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
        Eigen::Matrix<double, 1, 5> distortion_coefficients(distortion_coefficients_data.data());

        // Parse rectification matrix from YAML
        std::vector<double> rectification_matrix_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
        Eigen::Matrix<double, 3, 3> rectification_matrix(rectification_matrix_data.data());

        // Parse projection matrix from YAML
        std::vector<double> projection_matrix_data = config["projection_matrix"]["data"].as<std::vector<double>>();
        Eigen::Matrix<double, 3, 4> projection_matrix(projection_matrix_data.data());
        
        //[K1,K2,P1,P2,K3] --> [K1,K2,K3,P1,P2]
        auto tmp = distortion_coefficients[4];
        distortion_coefficients[4] = distortion_coefficients[3];
        distortion_coefficients[3] = distortion_coefficients[2];
        distortion_coefficients[2] = tmp;

        eigen2cv(camera_matrix,intrinsic);
        eigen2cv(distortion_coefficients, dis_coeff);

        return true;
    }
}


