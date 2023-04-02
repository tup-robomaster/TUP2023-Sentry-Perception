#include "../../global_user/include/global_user/global_user.hpp"
#include "./armor_detector/armor_detector.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>

//depthai msgs
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <depthai_ros_msgs/msg/spatial_detection.hpp>

//tf2
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
//custom message
#include "global_interface/msg/detection_array.hpp"
using namespace global_user;
using namespace coordsolver;
namespace perception_detector
{
    class DetectorNode : public rclcpp::Node
    {
        typedef global_interface::msg::DetectionArray DetectionArrayMsg;

    public:
        DetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~DetectorNode();
        
        bool initParams();
        bool updateParams();

        global_interface::msg::Detection armor2Detection(Armor armor, std_msgs::msg::Header header);
        std::vector<Armor> detectionArray2Armors(global_interface::msg::DetectionArray detections);
        Armor detection2Armor(global_interface::msg::Detection detection);
        std::vector<visualization_msgs::msg::Marker> getvisRobotMarkers(global_interface::msg::DetectionArray detections);
        bool sphereNMS(std::vector<Armor> &armors);
    private:
        rclcpp::Time time_start_;
        ImageInfo image_info_;
        ImageSize image_size_;

        std::vector<std::shared_ptr<image_transport::Subscriber>> img_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::deque<global_interface::msg::DetectionArray> detections_deque_;
        // Pub target armor msg.
        rclcpp::Publisher<DetectionArrayMsg>::SharedPtr perception_info_pub_;
    private:    
        // Params callback.
        // Subscribe img. 
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        void postProcessCallback();
        void depthaiNNCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr spatial_detections);
        // Subscribe serial msg.
        Mutex msg_mutex_;

        
    public:
        std::vector<Robot> robot_results;
        std::vector<Robot> final_robot_results;
        std::vector<std::string> registered_cams;
        Mutex param_mutex_;
        Mutex detections_mutex_;
        DetectorParam detector_params_;
        PathParam path_params_;
        DebugParam debug_;
        rclcpp::TimerBase::SharedPtr postprocess_timer_;
        std::vector<std::unique_ptr<Detector>> detectors_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_robot_pub_;
        rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr depthai_sub_;
    };
} //namespace detector