#include "../../include/armor_detector/armor_detector.hpp"

using namespace std;

namespace perception_detector
{
    Detector::Detector(const PathParam& path_param, const DetectorParam& detector_params, const DebugParam& debug_params) 
    : detector_params_(detector_params),
    TRTinfer_(0),
    path_params_(path_param), debug_params_(debug_params), logger_(rclcpp::get_logger("armor_detector"))
    {
        //初始化Infer
        std::string onnx_path = path_param.network_path;
        int postfix_idx = onnx_path.find(".onnx");
        if (postfix_idx == onnx_path.npos)
        {
            RCLCPP_ERROR(logger_, "Invalid weight file,please check your file");
            throw std::exception();
        }
        std::string trt_path = onnx_path;
        trt_path.replace(trt_path.begin()+postfix_idx, trt_path.begin()+postfix_idx+5, ".engine");
        if (!TRTinfer_.initMoudle(trt_path, 1, 4, 8, 8, 128))
        {
            RCLCPP_WARN(logger_,"Invalid trt file, attempting to convert onnx to trt.");
            //TODO: Adaptive Batchsize
            nvinfer1::IHostMemory *data = TRTinfer_.createEngine(onnx_path, 1, 640, 640);
            RCLCPP_INFO(logger_,"TRT Engine successfully converted");
            TRTinfer_.saveEngineFile(data, trt_path);
            RCLCPP_INFO(logger_,"TRT Engine successfully saved to %s", trt_path.c_str());
            TRTinfer_.initMoudle(trt_path, 1, 4, 8, 8, 128);
        }
        else
        {
            RCLCPP_WARN(logger_,"Using cached TRT engine file,do remember to delete it before you use different onnx.");
        }

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
    bool Detector::detect(cv::Mat &src, std::vector<Armor> &armors, int cam_idx)
    {
        std::vector<cv::Mat> inputs;
        std::vector<DetectObject> objects;
        inputs.push_back(src);
        objects = TRTinfer_.doInference(inputs,0.7,0.3)[0];
        // std::cout<<i<<".SIZE:"<<objects.size()<<std::endl;
        cv::Mat intrinsic = intrinsics[cam_idx].intrinsic;
        cv::Mat dis_coeff = intrinsics[cam_idx].dis_coeff;
        //生成装甲板对象
        for (auto object : objects)
        {
            Armor armor;
            if (object.cls == 6 || object.cls == 7)
                continue;
            if (object.cls == 0)
                object.cls = 7;
            armor.id = object.cls;
            // RCLCPP_INFO(logger_, "armor id:%d",armor.id );
            armor.color = object.color / 2;
            armor.conf = object.prob;

            if (armor.color == 0)
                armor.key = "B" + to_string(object.cls);
            if (armor.color == 1)
                armor.key = "R" + to_string(object.cls);
            if (armor.color == 2)
                armor.key = "N" + to_string(object.cls);
            if (armor.color == 3)
                armor.key = "P" + to_string(object.cls);

            if (object.color >= 4)
                continue;
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
            armor.rmat = rmat_eigen;
            armor.area = object.area;
            armors.push_back(armor);
        }
        if (armors.empty())
            return false;
        else
            return true;
    }

    bool Detector::addCameraIntrinsicsByYAML(const std::string& yaml_file_path)
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

        cv::Mat intrinsic;
        cv::Mat dis_coeff;
        eigen2cv(camera_matrix,intrinsic);
        eigen2cv(distortion_coefficients, dis_coeff);

        CvIntrinsic cv_intrinsic;
        cv_intrinsic.intrinsic = intrinsic;
        cv_intrinsic.dis_coeff = dis_coeff;

        intrinsics.push_back(cv_intrinsic);

        return true;
    }
}


