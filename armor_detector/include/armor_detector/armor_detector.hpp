//C++
#include <iostream>
#include <future>
#include <algorithm>
#include <vector>
#include <string>
#include <atomic>

//ros
#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../inference/inference_api2.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/autoaim.hpp"

using namespace global_user;
using namespace coordsolver;
using namespace cv;
using namespace std;
namespace perception_detector
{
    enum Color 
    {
        BLUE,
        RED
    };
    struct Armor : ObjectBase
    {
        int area;
        Rect roi;
        Rect rect;
        Point2f apex2d[4];
        RotatedRect rrect;
        cv::Point2d center2d;
        TargetType type;
    };
    struct Robot 
    {
        cv::Rect m_rect;
        int m_number;
        int m_color;
        // int m_armornums;
        int m_id; 
        // cv::Point3f XYZ_camera;
        // cv::Point3f XYZ_world;
        Eigen::Vector3d XYZ_camera;
        Eigen::Vector3d XYZ_world;
        // Robot()
        // {
        //     m_number = 0;
        //     m_color = 0;
        //     // m_armornums = 0;
        //     m_id = -1;
        //     XYZ_camera = {0.0, 0.0, 0.0};
        //     XYZ_world = {0.0, 0.0, 0.0};
        // }
    };
    struct DebugParam
    {
        // bool debug_without_com;
        // bool using_imu;
        // bool using_roi;
        bool show_aim_cross;
        bool show_img;
        bool detect_red;
        bool show_all_armors;
        bool show_fps;
        // bool print_letency;
        // bool print_target_info;
        // bool save_data;
        // bool save_dataset;

        DebugParam()
        {
            // debug_without_com = true;
            // using_imu = false;
            // using_roi = false;
            show_aim_cross = false;
            show_img = true;
            detect_red = true;
            show_all_armors = true;
            show_fps = true;
            // print_letency = false;
            // print_target_info = true; 
            // save_data = false;
            // save_dataset = false;
        }
    };

    struct PathParam
    {
        std::string camera_param_path;
        std::string network_path;
    };

    struct DetectorParam
    {
        int armor_type_wh_thres; //大小装甲板长宽比阈值
        int max_armors_cnt;    //视野中最多装甲板数
        double armor_roi_expand_ratio_width;
        double armor_roi_expand_ratio_height;
        double armor_conf_high_thres;

        Color color;
        DetectorParam()
        {
            color = RED;
            armor_type_wh_thres = 3;
            // max_lost_cnt = 5;
            max_armors_cnt = 8;
            // max_v = 0;
            // no_crop_thres = 2e-3;
            // no_crop_ratio = 2e-3;
            // full_crop_ratio = 1e-4;

            // hero_danger_zone = 4.0;
            armor_roi_expand_ratio_width = 1.1;
            armor_roi_expand_ratio_height = 1.5;
            armor_conf_high_thres = 0.82;
        }
    };

    // std::vector<cv::Mat> _intrinsic_cvs;
    // std::vector<cv::Mat> _extrinsic_cvs;
    // std::vector<cv::Mat> _discoff_cvs;

    class Detector
    {
    public:
        Detector(const PathParam& path_params, const DetectorParam& detector_params_, const DebugParam& debug_params_);
        ~Detector();

        // void run();
        bool detect(cv::Mat &src, std::vector<Armor> &armors);
        bool setCameraIntrinsicsByYAML(const std::string& yaml_file_path);
        // void RobotMatch(const int id, const std::vector<Armor> &results, std::vector<Robot> &Robots);
        // void robot_detect(TaskData& src);


        // void RobotMatch(const int id, const std::vector<Armor> &results, std::vector<Robot> &Robots);

        // bool gyro_detector(TaskData &src, global_interface::msg::Autoaim& target_info);

        // Point2i cropImageByROI(Mat &img);
        // ArmorTracker* chooseTargetTracker(vector<ArmorTracker*> trackers, double timestamp);
        // int chooseTargetID(vector<Armor> &armors, double timestamp);
   
    public:
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

        std::vector<Armor> last_armors;
        cv::Mat intrinsic;
        cv::Mat dis_coeff;
        // std::vector<Robot> robot_results;
        // std::vector<Robot> final_robot_results;

        bool is_init_;
        ofstream data_save_;
        ArmorDetector armor_detector_;
    private:
        Armor last_armor;
        std::vector<ArmorObject> objects;
        std::map<string, int> new_armors_cnt_map;    //装甲板计数map，记录新增装甲板数
        rclcpp::Logger logger_;
        ofstream file_;
        std::string path_prefix_ = "src/camera_driver/recorder/autoaim_dataset/";

    private:
        int count;
        rclcpp::Time time_start;
        rclcpp::Time time_infer;
        rclcpp::Time time_crop;
        
        double timestamp;
        int dead_buffer_cnt;

        bool is_last_target_exists;
        bool is_target_switched;
        int lost_cnt;
        double last_timestamp; //当前帧时间戳
        double prev_timestamp; //上一帧时间戳
        double last_target_area;
        double last_bullet_speed;
        Point2i last_roi_center;
        Eigen::Vector3d last_aiming_point;
        
        Point2i roi_offset;
        Size2i input_size;

        void showArmors(cv::Mat &src, std::vector<Armor> armors);
        void showCar(TaskData& src);
        
    private:
        double cur_period_;
        double last_period_;
        deque<double> history_period_;
        deque<double> new_period_deq_;
        double last_ave_period_;
        double cur_ave_period_;

    public:
        //Debug param.
        DebugParam debug_params_;
        PathParam path_params_;
        DetectorParam detector_params_;
    };
} //namespace detector