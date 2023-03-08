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

//custom message
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/autoaim.hpp"

using namespace global_user;
using namespace coordsolver;
namespace armor_detector
{
    class DetectorNode : public rclcpp::Node
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;

    public:
        DetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~DetectorNode();
        
        void detect(TaskData& src);
    private:
        rclcpp::Time time_start_;
        ImageInfo image_info_;
        ImageSize image_size_;

        // Pub target armor msg.
        rclcpp::Publisher<AutoaimMsg>::SharedPtr armor_info_pub_;
    private:    
        // Params callback.
        bool updateParam();
        
    private:
        // Subscribe img. 
        std::shared_ptr<image_transport::Subscriber> img_sub_;
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);

        // Subscribe serial msg.
        Mutex msg_mutex_;

        
    public:
        Mutex param_mutex_;
        DetectorParam detector_params_;
        // GyroParam gyro_params_;
        PathParam path_params_;
        DebugParam debug_;

        std::unique_ptr<Detector> detector_;
        std::unique_ptr<Detector> initDetector();

    protected:
        bool using_shared_memory_;
        SharedMemoryParam shared_memory_param_;
        std::thread read_memory_thread_; //共享内存读线程
        void threadCallback();
    };
} //namespace detector