#include "../include/detector_node.hpp"

using namespace std::placeholders;
namespace armor_detector
{
    DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
    {
        RCLCPP_WARN(this->get_logger(), "Starting detector node...");

        try
        {   //detector类初始化
            this->detector_ = initDetector();
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(this->get_logger(), "Fatal while initializing detector class: %s", e.what());
        }

        if(!detector_->is_init_)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing network model...");
            detector_->armor_detector_.initModel(path_params_.network_path);
            // RCLCPP_INFO(this->get_logger(), "initmodel end...");
            // cout<<"camera_param_path:"<<path_params_.camera_param_path<<endl;
            detector_->coordsolver_.loadParam(path_params_.camera_param_path, path_params_.camera_name);
            detector_->is_init_ = true;
            // RCLCPP_INFO(this->get_logger(), "loadparam end...");

        }
        RCLCPP_INFO(this->get_logger(), "Initialize network model end...");
        time_start_ = detector_->steady_clock_.now();

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();
        
        // target info pub.
        armor_info_pub_ = this->create_publisher<AutoaimMsg>("/armor_info", qos);

        // CameraType camera_num;
        this->declare_parameter<int>("camera_num", 1);
        int camera_num = this->get_parameter("camera_num").as_int();

        // Subscriptions transport type.
        std::string transport_type = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        
        image_size_ = image_info_.image_size_map[camera_num];
        // image sub.
        // RCLCPP_INFO(this->get_logger(), "start image callback...");
        std::string camera_topic = image_info_.camera_topic_map[camera_num];
        img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, camera_topic,
            std::bind(&DetectorNode::imageCallback, this, _1), transport_type));
    }

    DetectorNode::~DetectorNode()
    {
       
    }

    void DetectorNode::detect(TaskData& src)
    {
        auto img_sub_time = detector_->steady_clock_.now();
        src.timestamp = (img_sub_time - time_start_).nanoseconds();
        
        AutoaimMsg target_info;
        bool is_target_lost = true;
        param_mutex_.lock();
        if(detector_->armor_detect(src, is_target_lost))
        {   
            RCLCPP_INFO(this->get_logger(), "armors detector...");

        }
        param_mutex_.lock();
        target_info.is_target_lost = is_target_lost;
        
        // Publish target's information containing 3d point and timestamp.
        armor_info_pub_->publish(std::move(target_info));
        
        debug_.show_img = this->get_parameter("show_img").as_bool();
        if(debug_.show_img)
        {
            // RCLCPP_INFO(this->get_logger(), "show img...");
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
    }

    /**
     * @brief 图像数据回调
     * 
     * @param img_info 图像传感器数据
     */
    void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        TaskData src;
        std::vector<Armor> armors;

        if(!img_info)
            return;
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);

        //目标检测接口函数
        detect(src);
    }

    /**
     * @brief 初始化detector类
     * 
     * @return std::unique_ptr<Detector> 
     */
    std::unique_ptr<Detector> DetectorNode::initDetector()
    {
        //Detector params.
        this->declare_parameter<int>("armor_type_wh_thres", 3);//大小装甲板长宽比阈值
        // this->declare_parameter<int>("max_lost_cnt", 5);//最大丢失目标帧数
        this->declare_parameter<int>("max_armors_cnt", 8);//视野中最多装甲板数
        // this->declare_parameter<int>("max_v", 8);
        // this->declare_parameter<double>("no_crop_thres", 1e-2);
        // this->declare_parameter<int>("hero_danger_zone", 4);
        this->declare_parameter<bool>("color", true);
        // this->declare_parameter<double>("no_crop_ratio", 2e-3);
        // this->declare_parameter<double>("full_crop_ratio", 1e-4);
        this->declare_parameter<double>("armor_roi_expand_ratio_width", 1.1);
        this->declare_parameter<double>("armor_roi_expand_ratio_height", 1.5);
        this->declare_parameter<double>("armor_conf_high_thres", 0.82);
        
        //TODO:Set by your own path.
        this->declare_parameter("camera_name", "KE0200110075"); //相机型号
        this->declare_parameter("camera_param_path", "src/armor_detector/config/camera_info.yaml");
        this->declare_parameter("network_path", "src/armor_detector/model/opt-0527-002.xml");
        // this->declare_parameter("save_path", "src/data/old_infer1_2.txt");
        
        //Debug.
        // this->declare_parameter("debug_without_com", true);
        // this->declare_parameter("using_imu", false);
        // this->declare_parameter("using_roi", true);
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_img",false);
        this->declare_parameter("detect_red", true);
        this->declare_parameter("show_fps", false);
        // this->declare_parameter("print_letency", false);
        // this->declare_parameter("print_target_info", false);
        this->declare_parameter("show_all_armors", false);
        // this->declare_parameter("save_data", false);
        // this->declare_parameter("save_dataset", false);
        
        //Update param from param server.
        updateParam();

        return std::make_unique<Detector>(path_params_, detector_params_, debug_);
    }

    /**
     * @brief 更新参数
     * 
     * @return true 
     * @return false 
     */
    bool DetectorNode::updateParam()
    {
        detector_params_.armor_type_wh_thres = this->get_parameter("armor_type_wh_thres").as_int();
        // detector_params_.max_lost_cnt = this->get_parameter("max_lost_cnt").as_int();
        detector_params_.max_armors_cnt = this->get_parameter("max_armors_cnt").as_int();
        // detector_params_.max_v = this->get_parameter("max_v").as_int();
        // detector_params_.no_crop_thres = this->get_parameter("no_crop_thres").as_double();
        // detector_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        bool det_red = this->get_parameter("color").as_bool();
        if(det_red)
            detector_params_.color = RED;
        else
            detector_params_.color = BLUE;
        // detector_params_.no_crop_ratio = this->get_parameter("no_crop_ratio").as_double();
        // detector_params_.full_crop_ratio = this->get_parameter("full_crop_ratio").as_double();
        detector_params_.armor_roi_expand_ratio_width = this->get_parameter("armor_roi_expand_ratio_width").as_double();
        detector_params_.armor_roi_expand_ratio_height = this->get_parameter("armor_roi_expand_ratio_height").as_double();
        detector_params_.armor_conf_high_thres = this->get_parameter("armor_conf_high_thres").as_double();

        debug_.detect_red = this->get_parameter("detect_red").as_bool();
        // debug_.debug_without_com  = this->get_parameter("debug_without_com").as_bool();
        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        // debug_.using_imu = this->get_parameter("using_imu").as_bool();
        // debug_.using_roi = this->get_parameter("using_roi").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        // debug_.print_letency = this->get_parameter("print_letency").as_bool();
        // debug_.print_target_info = this->get_parameter("print_target_info").as_bool();
        debug_.show_all_armors = this->get_parameter("show_all_armors").as_bool();
        // debug_.save_data = this->get_parameter("save_data").as_bool();
        // debug_.save_dataset = this->get_parameter("save_dataset").as_bool();

        path_params_.camera_name = this->get_parameter("camera_name").as_string();
        path_params_.camera_param_path = this->get_parameter("camera_param_path").as_string();
        path_params_.network_path = this->get_parameter("network_path").as_string();
        // path_params_.save_path = this->get_parameter("save_path").as_string();

        return true;
    }
} //namespace detector

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_detector::DetectorNode>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::DetectorNode)