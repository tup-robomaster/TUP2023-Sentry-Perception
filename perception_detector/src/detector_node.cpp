#include "../include/detector_node.hpp"

using namespace std::placeholders;
namespace perception_detector
{
    DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("perception_detector", options)
    {
        RCLCPP_WARN(this->get_logger(), "Starting detector node...");
        initParams();

        img_sub_.clear();
        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();
        
        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // target info pub.
        std::string transport_type = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        perception_info_pub_ = this->create_publisher<DetectionArrayMsg>("perception_detector/perception_array", qos);
        vis_robot_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("perception_detector/visual_robot", qos);
        postprocess_timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&DetectorNode::postProcessCallback, this));
        


        // CameraType camera_num;
        std::cout<<path_params_.camera_param_path<<std::endl;
        YAML::Node config = YAML::LoadFile(path_params_.camera_param_path);
        const YAML::Node& top_node = config["cams"];
        // 获取本级列表并遍历所有元素
        YAML::const_iterator it = top_node.begin();
        std::vector<std::shared_future<bool>> detector_init_task;
        auto init_func = [=](std::string camera_topic, std::string camera_info_path)
                            {
                                auto detector = std::make_unique<Detector>(path_params_, detector_params_, debug_);
                                detector->setCameraIntrinsicsByYAML(camera_info_path);
                                img_sub_.push_back(std::make_shared<image_transport::Subscriber>
                                                    (image_transport::create_subscription(this, camera_topic,
                                                    std::bind(&DetectorNode::imageCallback, this, _1), transport_type, rmw_qos)));
                                detectors_.push_back(std::move(detector));
                                return true;
                            };

        for (int i = 0; i < top_node.size(); ++i)
        {
            // Get key name
            const std::string key = it->first.as<std::string>();
            // Get sub-node for key
            const YAML::Node& sub_node = it->second;
            // Loop over keys in sub-node
            std::string camera_topic = sub_node["image_topic"].as<std::string>();
            std::string camera_frame = sub_node["frame_id"].as<std::string>();
            std::string camera_info_path = sub_node["camera_info_path"].as<std::string>();
            registered_cams.push_back(camera_frame);
            detector_init_task.push_back(std::async(std::launch::async, init_func, camera_topic, camera_info_path));
            RCLCPP_INFO(this->get_logger(), "Registered callback for %s ...", key.c_str());
            ++it;
        }
        //Wait for detectors to init...
        for (auto future : detector_init_task)
            future.wait();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    DetectorNode::~DetectorNode()
    {
       
    }
    void DetectorNode::postProcessCallback()
    {
        std::vector<global_interface::msg::DetectionArray> detections_vec;
        detections_mutex_.lock();
        // std::cout<<"vecsize1:"<<detections_deque_.size()<<std::endl;
        if (!detections_deque_.empty())
        {  
            std::sort(detections_deque_.begin(), detections_deque_.end(), [](const global_interface::msg::DetectionArray& a,
                                                                            const global_interface::msg::DetectionArray& b)
                                                                            {return (a.header.stamp.sec + 1e-9 * a.header.stamp.nanosec) <
                                                                            (b.header.stamp.sec + 1e-9 * b.header.stamp.nanosec);});
            while (detections_deque_.size() > 0)
            {
                auto first_element = detections_deque_.front();
                auto duration = rclcpp::Duration(this->now() - rclcpp::Time(first_element.header.stamp));
                if (duration < 300ms)
                {
                    break;
                }
                else
                {
                    detections_deque_.pop_front();
                }
            }
        }

        for (auto detections : detections_deque_)
        {
            detections_vec.push_back(detections);
        }
        detections_mutex_.unlock();
        if (!detections_vec.empty())
        {
            global_interface::msg::DetectionArray detections_msg;
            detections_msg.header= detections_vec.back().header;
            std::vector<Armor> armors;
            for (auto detections : detections_vec)
            {
                auto armors_tmp = detectionArray2Armors(detections);
                for (auto armor : armors_tmp)
                    armors.push_back(armor);
            }
            sphereNMS(armors);
            if (!armors.empty())
            {
                for (auto armor : armors)
                {
                    RCLCPP_INFO(this->get_logger(), "Detected %s @ %f, %f, %f",
                                armor.key.c_str(), armor.armor3d_cam[0], armor.armor3d_cam[1], armor.armor3d_cam[2]);
                    auto detection = armor2Detection(armor, detections_msg.header);
                    detections_msg.detections.push_back(detection);
                }
                visualization_msgs::msg::MarkerArray vis_array;
                auto vis_markers = getvisRobotMarkers(detections_msg);
                for (auto marker : vis_markers)
                    vis_array.markers.push_back(marker);
                vis_robot_pub_->publish(vis_array);
                //Publish msg
                perception_info_pub_->publish(detections_msg);
            }
        }
    }

    global_interface::msg::Detection DetectorNode::armor2Detection(Armor armor, std_msgs::msg::Header header)
    {
        global_interface::msg::Detection detection;
        detection.header = header;
        detection.conf = armor.conf;
        detection.type = armor.key;
        detection.center.position.x = armor.armor3d_cam[0];
        detection.center.position.y = armor.armor3d_cam[1];
        detection.center.position.z = armor.armor3d_cam[2];
        // std::cout<<armor.rmat<<std::endl;
        Eigen::Quaterniond quat(armor.rmat);
        detection.center.orientation.x = quat.x();
        detection.center.orientation.y = quat.y();
        detection.center.orientation.z = quat.z();
        detection.center.orientation.w = quat.w();
        // std::cout<<armor.key<<std::endl;
        return detection;
    }

    std::vector<visualization_msgs::msg::Marker> DetectorNode::getvisRobotMarkers
                                    (global_interface::msg::DetectionArray detections)
    {
        std::vector<visualization_msgs::msg::Marker> vis_markers;
        for (auto detection : detections.detections)
        {
            visualization_msgs::msg::Marker vis_marker;
            visualization_msgs::msg::Marker vis_marker_text;
            vis_marker.header = detections.header;
            vis_marker.ns = detection.type + "_pos";
            vis_marker.id = 0;
            vis_marker.type = visualization_msgs::msg::Marker::SPHERE;
            vis_marker.action = visualization_msgs::msg::Marker::ADD;
            vis_marker.lifetime = rclcpp::Duration::from_seconds(1);
            vis_marker.pose.position.x = detection.center.position.x;
            vis_marker.pose.position.y = detection.center.position.y;
            vis_marker.pose.position.z = detection.center.position.z;
            vis_marker.pose.orientation.x = detection.center.orientation.x;
            vis_marker.pose.orientation.y = detection.center.orientation.y;
            vis_marker.pose.orientation.z = detection.center.orientation.z;
            vis_marker.pose.orientation.w = detection.center.orientation.w;
            vis_marker.scale.x = 0.5;
            vis_marker.scale.y = 0.5;
            vis_marker.scale.z = 0.5;
            vis_marker.color.a = 1.0;

            //Context will be different for marker and text marker below,copy here.
            vis_marker_text = vis_marker;
            vis_marker_text.ns = detection.type + "_label";
            vis_marker_text.pose.position.z = vis_marker_text.pose.position.z + 0.5;
            vis_marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            vis_marker_text.text = detection.type;
            vis_marker_text.color.r = 1.0;
            vis_marker_text.color.g = 1.0;
            vis_marker_text.color.b = 1.0;

            if (detection.type[0] == 'R')
            {
                vis_marker.color.r = 1.0;
                vis_marker.color.g = 0.2;
                vis_marker.color.b = 0.2;
            }
            else if (detection.type[0] == 'B')
            {
                vis_marker.color.r = 0.2;
                vis_marker.color.g = 0.2;
                vis_marker.color.b = 1.0;
            }
            else if (detection.type[0] == 'N')
            {
                vis_marker.color.r = 1.0;
                vis_marker.color.g = 1.0;
                vis_marker.color.b = 1.0;
            }
            else if (detection.type[0] == 'P')
            {
                vis_marker.color.r = 1.0;
                vis_marker.color.g = 0.0;
                vis_marker.color.b = 1.0;
            }

            vis_markers.push_back(vis_marker);
            vis_markers.push_back(vis_marker_text);
        }
        return vis_markers;
    }

    Armor DetectorNode::detection2Armor(global_interface::msg::Detection detection)
    {
        Armor armor;
        armor.conf = detection.conf;
        armor.key = detection.type;
        armor.armor3d_cam[0] = detection.center.position.x;
        armor.armor3d_cam[1] = detection.center.position.y;
        armor.armor3d_cam[2] = detection.center.position.z;
        Eigen::Quaterniond quat(detection.center.orientation.w,
                        detection.center.orientation.x,
                        detection.center.orientation.y,
                        detection.center.orientation.z);
        armor.rmat = quat.toRotationMatrix();
        return armor;
    }

    std::vector<Armor> DetectorNode::detectionArray2Armors(global_interface::msg::DetectionArray detections)
    {
        std::vector<Armor> armors;
        std::vector<global_interface::msg::Detection> detection_vec = detections.detections;
        for (auto detection : detections.detections)
        {
            auto armor = detection2Armor(detection);
            armors.push_back(armor);
        }
        return armors;

    }

    bool DetectorNode::sphereNMS(std::vector<Armor> &armors)
    {
        //First Stage NMS:Armor
        double max_sphere_dist = 0.5f;
        std::map<int, bool> overlap_map;
        std::map<int, std::vector<int>> overlap_idx_map;
        struct ArmorWithOverlapCnt
        {
            Armor armor;
            int overlap_cnt;
        };
        std::vector<ArmorWithOverlapCnt> first_nms_out;
        std::vector<Armor> second_nms_out;
        //Get overlap status.
        for (int i = 0; i < armors.size(); i++)
        {
            for (int j = i+1; j< armors.size(); j++)
            {
                if (overlap_map[j])
                    continue;
                auto armor1 = armors[i];
                auto armor2 = armors[j];
                double sphere_dist = (armor1.armor3d_cam - armor2.armor3d_cam).norm();
                if (sphere_dist < max_sphere_dist && armor1.key == armor2.key)
                {
                    overlap_map[j] = true;
                    overlap_idx_map[i].push_back(j);
                }
            }
        }
        for (int i = 0; i < armors.size(); i++)
        {
            if (!overlap_map[i])
            {
                Armor armor = armors[i];
                //Skip armor with no overlap armor,which might be a FN/FP sample.
                if (overlap_idx_map[i].size() != 0)
                {
                    int overlap_cnt = overlap_idx_map[i].size();
                    for (auto overlap_cnt : overlap_idx_map[i])
                    {
                        armor.armor3d_cam=armor.armor3d_cam + armors[overlap_cnt].armor3d_cam;
                    }
                    armor.armor3d_cam /= (overlap_cnt + 1);
                    ArmorWithOverlapCnt armor_with_overlap_cnt = {armor, overlap_cnt};
                    first_nms_out.push_back(armor_with_overlap_cnt);
                }
            }
        }


        //Second Stage NMS:Robot
        //Sort by overlap cnt.
        std::sort(first_nms_out.begin(), first_nms_out.end(), [](const ArmorWithOverlapCnt& a,
                                                                            const ArmorWithOverlapCnt& b)
                                                                            {return a.overlap_cnt > b.overlap_cnt;});
        std::map<std::string, bool> is_key_appeared;
        for (int i = 0; i < first_nms_out.size(); i++)
        {
            Armor armor = first_nms_out[i].armor;
            if (!is_key_appeared[armor.key])
            {
                second_nms_out.push_back(armor);
                is_key_appeared[armor.key] = true;
            }
        }
        armors = second_nms_out;
        return true;
    }

    /**
     * @brief 图像数据回调
     * @param img_info 图像传感器数据
     */
    void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        std::string frame_id = img_info->header.frame_id;
        //获取该相机所分配的detector idx.
        auto cam_it = std::find(registered_cams.begin(), registered_cams.end(), frame_id);
        if (cam_it != registered_cams.end())
        {
            int idx = cam_it - registered_cams.begin();
            std::vector<Armor> armors;
            if (detectors_.at(idx)->detect(img, armors))
            {
                //Transform from cam frame to base link.
                geometry_msgs::msg::TransformStamped tf_msg;
                try
                {
                    tf_msg = tf_buffer_->lookupTransform("base_link", frame_id, img_info->header.stamp,
                                                                         rclcpp::Duration::from_seconds(0.1));
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
                    return;
                }
                //Transform detection.
                std_msgs::msg::Header header = img_info->header;
                global_interface::msg::DetectionArray detections;
                detections.header = img_info->header;
                detections.header.frame_id = "base_link";
                for (auto armor : armors)
                {
                    auto detection = armor2Detection(armor, header);
                    auto detection_transformed = detection;
                    geometry_msgs::msg::PoseStamped pose, transformed_pose;
                    pose.header = detection.header;
                    pose.pose = detection.center;
                    try {
                        tf2::doTransform(pose, transformed_pose, tf_msg);
                    } catch (tf2::TransformException &ex) {
                        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
                        return;
                    }
                    detection_transformed.header.frame_id = "base_link";
                    detection_transformed.center = transformed_pose.pose;
                    detections.detections.push_back(detection_transformed);
                }

                detections_mutex_.lock();
                detections_deque_.push_back(detections);
                detections_mutex_.unlock();

                debug_.show_img = this->get_parameter("show_img").as_bool();
                if(debug_.show_img)
                {
                    // RCLCPP_INFO(this->get_logger(), "show img...");
                    cv::namedWindow(frame_id + "_detect", cv::WINDOW_AUTOSIZE);
                    cv::imshow(frame_id + "_detect", img);
                    cv::waitKey(1);
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Detected invalid frame_id for detect...");
        }
    }

    /**
     * @brief 初始
     * 
     * @return bool
     */
    bool DetectorNode::initParams()
    {
        //Detector params.
        this->declare_parameter<int>("armor_type_wh_thres", 3);//大小装甲板长宽比阈值
        this->declare_parameter<double>("armor_roi_expand_ratio_width", 1.1);
        this->declare_parameter<double>("armor_roi_expand_ratio_height", 1.5);
        this->declare_parameter<double>("armor_conf_high_thres", 0.82);
        
        this->declare_parameter("camera_param_path", "/config/config.yaml");
        this->declare_parameter("network_path", "/model/opt-0527-002.xml");
        //Debug.
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_img", true);
        this->declare_parameter("show_fps", false);
        this->declare_parameter("show_all_armors", true);
        
        //Update param from param server.
        updateParams();

        return true;
    }

    /**
     * @brief 更新参数
     * 
     * @return true 
     * @return false 
     */
    bool DetectorNode::updateParams()
    {
        detector_params_.armor_type_wh_thres = this->get_parameter("armor_type_wh_thres").as_int();
        detector_params_.armor_roi_expand_ratio_width = this->get_parameter("armor_roi_expand_ratio_width").as_double();
        detector_params_.armor_roi_expand_ratio_height = this->get_parameter("armor_roi_expand_ratio_height").as_double();
        detector_params_.armor_conf_high_thres = this->get_parameter("armor_conf_high_thres").as_double();

        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        debug_.show_all_armors = this->get_parameter("show_all_armors").as_bool();
        string pkg_share_pth = ament_index_cpp::get_package_share_directory("perception_detector");
        path_params_.camera_param_path = pkg_share_pth + this->get_parameter("camera_param_path").as_string();
        path_params_.network_path = pkg_share_pth + this->get_parameter("network_path").as_string();

        return true;
    }
} //namespace detector

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perception_detector::DetectorNode>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_detector::DetectorNode)