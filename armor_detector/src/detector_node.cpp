#include "../include/detector_node.hpp"

using namespace std::placeholders;
namespace armor_detector
{

    DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
    {
        RCLCPP_WARN(this->get_logger(), "Starting detector node...");

        // 问题：try-catch块中的代码可能会抛出异常，如果抛出异常，将会导致detector_指针为空，可能会导致程序崩溃。
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
            detector_->coordsolver_.loadParam(path_params_.camera_param_path, path_params_.camera_name);
            detector_->is_init_ = true;

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
        for (int i = 1; i <= camera_num; i++){
            RCLCPP_INFO(this->get_logger(), "start image%d callback...",i);
            std::string camera_topic = image_info_.camera_topic_map[camera_num];
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, camera_topic,
                std::bind(&DetectorNode::imageCallback, this, _1), transport_type));
        }
    }

    DetectorNode::~DetectorNode()
    {
       
    }

    // 用于存储测量组的数据，包括三个图像的时间和三个相机检测到的装甲板信息。
    struct MeasureGroup
    {
        double img1_time;
        double img2_time;
        double img3_time;

        std::vector<Armor> cam1_objects;
        std::vector<Armor> cam2_objects;
        std::vector<Armor> cam3_objects;
    };
    MeasureGroup Measures;

    // bool sync_packages(MeasureGroup meas, std::vector<std::vector<Armor>> &detected_objects)
    // {
    //     std::vector<std::vector<Armor>> detected_objects_temp;
    //     detected_objects_temp.push_back(meas.cam1_objects);
    //     detected_objects_temp.push_back(meas.cam2_objects);
    //     detected_objects_temp.push_back(meas.cam3_objects);

    //     std::vector<double> src_times;
    //     src_times.push_back(meas.img1_time);
    //     src_times.push_back(meas.img2_time);
    //     src_times.push_back(meas.img3_time);

    //     bool syncFlag = false;

    //     for (int i = 0; i < src_times.size(); i++)
    //     {
    //         if (src_times[i] != 0 && () < 0.5)
    //         {
    //             detected_objects.push_back(detected_objects_temp[i]);
    //             syncFlag = true;
    //         }
    //         else
    //         {
    //             std::vector<Armor> emptyObject;            
    //             detected_objects.push_back(emptyObject);
    //             // RCLCPP_WARN(this->get_logger(), "Camera [%d] get image failed ..",i + 1);
    //         }
    //     }
    //     if (!syncFlag)
    //     {
    //         // RCLCPP_WARN(this->get_logger(), "image failed ..");
    //     }
    //     return syncFlag;
    // }
    // void DetectorNode::PublishDetectRobots(rclcpp::Publisher robot_detector_pub, std::vector<Robot> robot_results)
    // {
    //     robot_msgs::RobotLocation robots_temp;
    //     robot_msgs::GlobalMap submap_msg;
    //     submap_msg.header.stamp = ros::Time::now();

    //     for (int i = 0; i < robot_results.size(); ++i)
    //     {
    //         robots_temp.id = robot_results[i].m_id;
    //         robots_temp.x = robot_results[i].XYZ_body.x;
    //         robots_temp.y = robot_results[i].XYZ_body.y;
    //         robots_temp.yaw = 0;
    //         submap_msg.robots.push_back(robots_temp);
    //     }

    //     robot_detector_pub.publish(submap_msg);
    // }
    void DetectorNode::RobotMatch(int id, std::vector<Armor> &results, std::vector<Robot> &Robots)
    {
        std::vector<Armor> last;

        int numarmor = 0;
        std::vector<int> repeat_index;
        for (int i = 0; i < results.size(); i++)
        {
            Robot robot_temp ;
            bool flag = false;
            for(int j = i + 1; j < results.size(); j++)
            {
                if(results[i].id == results[j].id)
                {
                    float dis = sqrt(pow((results[i].armor3d_world[0] - results[j].armor3d_world[0]),2)+
                            pow((results[i].armor3d_world[1] - results[j].armor3d_world[1]),2));
                    if(dis < 0.5)
                    {
                        flag = true;
                        // repeat_index.push_back(i);
                        
                        robot_temp.m_id = results[i].id;
                        robot_temp.m_color = results[i].color;
                        robot_temp.m_rect.x = results[i].roi.x;
                        robot_temp.m_rect.y = results[i].roi.y;
                        robot_temp.XYZ_world[0] = (results[i].armor3d_world[0] + results[j].armor3d_world[1])/2;
                        robot_temp.XYZ_world[1] = (results[i].armor3d_world[1] + results[j].armor3d_world[1])/2;
                        robot_temp.m_rect.x = min(results[i].roi.x, results[j].roi.x);
                        robot_temp.m_rect.y = min(results[i].roi.y, results[j].roi.y);
                        robot_temp.m_number = min(results[i].conf, results[j].conf);
                        if(results[i].roi.x <= results[j].roi.x)
                            robot_temp.m_rect.width = results[j].roi.x + results[j].roi.width - results[i].roi.x ;
                        else
                            robot_temp.m_rect.width = results[i].roi.x + results[i].roi.width - results[j].roi.x ;
                        if(results[i].roi.y <= results[j].roi.y)
                            robot_temp.m_rect.height = results[j].roi.y + results[j].roi.height - results[i].roi.y ;
                        else
                            robot_temp.m_rect.height = results[i].roi.y + results[i].roi.height - results[j].roi.y ;
                        robot_results.push_back(robot_temp);
                        continue;
                    }
                }
            }
            if(!flag)
            {
                robot_temp.m_id = results[i].id;
                cout<<"armor:"<<robot_temp.m_id<<endl;
                robot_temp.m_color = results[i].color;
                robot_temp.m_number = results[i].conf;
                robot_temp.m_rect = results[i].roi;
                robot_temp.XYZ_world = results[i].armor3d_world;
                robot_temp.XYZ_camera = results[i].armor3d_cam;
                robot_results.push_back(robot_temp);
            }
        }
    }
    void DetectorNode::robot_detect(TaskData& src, std::vector<Armor> &armors)
    {
        std::vector<std::vector<Armor>> detected_objects;
        // std::vector<Robot> robot_result;
        // std::vector<Robot> final_robot_results;
        detected_objects.clear();
        detected_objects.push_back(armors);

        for(int i = 0 ; i < detected_objects.size(); i++)
        {
            // RCLCPP_INFO(this->get_logger(), "---------------------------");
            
            std::vector<Armor> obj = detected_objects[i];
            std::vector<Robot> robots;
            RobotMatch(i,obj,robots); 
            if(robot_results.empty())
            {
                RCLCPP_INFO(this->get_logger(), "No Detect Robot...");
            }
            for(int j = 0; j < robot_results.size();j++)
            {
                if(robot_results[j].m_id != 0)
                {
                    final_robot_results.push_back(robot_results[j]);
                }
            }
        }
        if (final_robot_results.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No Detect Final_Robot...");
        }
        else
        {
            final_robot_results.clear();
            final_robot_results = robot_results;

        }
    }



    void DetectorNode::detect(TaskData& src)
    {
        auto img_sub_time = detector_->steady_clock_.now();
        src.timestamp = (img_sub_time - time_start_).nanoseconds();
        std::vector<Armor> armors;
        
        AutoaimMsg target_info;
        bool is_target_lost = true;
        param_mutex_.lock();
        if(detector_->armor_detect(src, is_target_lost, armors))
        {   
            RCLCPP_INFO(this->get_logger(), "armors detector...");

        }
        param_mutex_.lock();
        // target_info.is_target_lost = is_target_lost;
        robot_detect(src, armors);
      
        // Publish target's information containing 3d point and timestamp.
        armor_info_pub_->publish(std::move(target_info));      
        
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

        debug_.show_img = this->get_parameter("show_img").as_bool();
        if(debug_.show_img)
        {
            // RCLCPP_INFO(this->get_logger(), "show img...");
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
        
    }
    void DetectorNode::imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        TaskData src2;
        std::vector<Armor> armors2;

        if(!img_info)
            return;
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src2.img);

        //目标检测接口函数
        detect(src2);

        debug_.show_img = this->get_parameter("show_img").as_bool();
        if(debug_.show_img)
        {
            // RCLCPP_INFO(this->get_logger(), "show img...");
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src2.img);
            cv::waitKey(1);
        }
    }
    void DetectorNode::imageCallback3(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        TaskData src3;
        std::vector<Armor> armors3;

        if(!img_info)
            return;
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src3.img);

        //目标检测接口函数
        detect(src3);

        debug_.show_img = this->get_parameter("show_img").as_bool();
        if(debug_.show_img)
        {
            // RCLCPP_INFO(this->get_logger(), "show img...");
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src3.img);
            cv::waitKey(1);
        }
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
        this->declare_parameter<int>("max_armors_cnt", 8);//视野中最多装甲板数
        this->declare_parameter<bool>("color", true);
        this->declare_parameter<double>("armor_roi_expand_ratio_width", 1.1);
        this->declare_parameter<double>("armor_roi_expand_ratio_height", 1.5);
        this->declare_parameter<double>("armor_conf_high_thres", 0.82);
        
        //TODO:Set by your own path.
        this->declare_parameter("camera_name", "KE0200110075"); //相机型号
        this->declare_parameter("camera_param_path", "src/armor_detector/config/camera_info.yaml");
        this->declare_parameter("network_path", "src/armor_detector/model/opt-0527-002.xml");
        
        //Debug.
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_img",false);
        this->declare_parameter("detect_red", true);
        this->declare_parameter("show_fps", false);
        this->declare_parameter("show_all_armors", false);
        
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
        detector_params_.max_armors_cnt = this->get_parameter("max_armors_cnt").as_int();
        bool det_red = this->get_parameter("color").as_bool();
        if(det_red)
            detector_params_.color = RED;
        else
            detector_params_.color = BLUE;
        detector_params_.armor_roi_expand_ratio_width = this->get_parameter("armor_roi_expand_ratio_width").as_double();
        detector_params_.armor_roi_expand_ratio_height = this->get_parameter("armor_roi_expand_ratio_height").as_double();
        detector_params_.armor_conf_high_thres = this->get_parameter("armor_conf_high_thres").as_double();

        debug_.detect_red = this->get_parameter("detect_red").as_bool();
        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        debug_.show_all_armors = this->get_parameter("show_all_armors").as_bool();


        path_params_.camera_name = this->get_parameter("camera_name").as_string();
        path_params_.camera_param_path = this->get_parameter("camera_param_path").as_string();
        path_params_.network_path = this->get_parameter("network_path").as_string();

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