#ifndef GLOBAL_USER_HPP_
#define GLOBAL_USER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>
#include <unistd.h>
#include <future>
#include <fstream>
#include <yaml-cpp/yaml.h>

//opencv
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Dense>
#include <Eigen/Core>

//linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

// daheng
#define DAHENG_IMAGE_WIDTH 1280 
#define DAHENG_IMAGE_HEIGHT 1024
// hik
#define HIK_IMAGE_WIDTH 1440     
#define HIK_IMAGE_HEIGHT 1080
// usb
#define USB_IMAGE_WIDTH 640     
#define USB_IMAGE_HEIGHT 480
// mvs
#define MVS_IMAGE_WIDTH 1280     
#define MVS_IMAGE_HEIGHT 1024

using namespace std;
namespace global_user
{   
    /**
     * @brief Global variables and funcs.
     * 
     */

    struct ImageSize
    {
        int width;
        int height;

        ImageSize()
        {
            this->width = DAHENG_IMAGE_WIDTH;
            this->height = DAHENG_IMAGE_HEIGHT;
        }
    };

    class ImageInfo
    {
    public:
        std::map<int, std::string> camera_topic_map;
        std::map<int, ImageSize> image_size_map;

        ImageInfo()
        {
            camera_topic_map = 
            {
                {0, "image_raw0"},
                {1, "image_raw"},
                {2, "image_raw2"},
                {3, "image_raw3"}
            };

            image_size_map[0].width = DAHENG_IMAGE_WIDTH;
            image_size_map[0].height = DAHENG_IMAGE_HEIGHT;
            image_size_map[1].width = HIK_IMAGE_WIDTH;
            image_size_map[1].height = HIK_IMAGE_HEIGHT;
            image_size_map[2].width = MVS_IMAGE_WIDTH;
            image_size_map[2].height = MVS_IMAGE_HEIGHT;
            image_size_map[3].width = USB_IMAGE_WIDTH;
            image_size_map[3].height = USB_IMAGE_HEIGHT;
        }
    };

    enum CameraType
    {
        DaHeng,
        HikRobot,
        MVSCam,
        USBCam,
    };

    enum TargetType 
    {  
        SMALL, 
        BIG, 
        // BUFF
    };

    struct TaskData
    {
        int mode;
        double bullet_speed;
        cv::Mat img;
        Eigen::Quaterniond quat;
        double timestamp; 
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    struct ObjectBase
    {
        int id;
        int color;
        double conf;
        std::string key;
        Eigen::Vector3d armor3d_cam;
        Eigen::Vector3d armor3d_world;
        Eigen::Vector3d euler;
        Eigen::Matrix3d rmat;
    };
    
    struct Object
    {
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    struct VideoRecordParam
    {
        std::future<void> writer;
        cv::VideoWriter video_recorder;
        std::string save_path;
        bool is_initialized;
        bool is_first_loop;
        int frame_cnt;
        int image_width;
        int image_height;

        VideoRecordParam()
        {
            save_path = "src/camera_driver/video/";
            is_initialized = false;
            is_first_loop = true;
            frame_cnt = 0;
            image_width = 1280; //FIXME:根据相机类型设置
            image_height = 1024;
        }
    };

    struct SharedMemoryParam
    {
        key_t key;               //生成一个key
        int shared_memory_id;    //共享内存的id
        void* shared_memory_ptr; //映射共享内存，得到虚拟地址
        SharedMemoryParam()
        {
            shared_memory_ptr = nullptr;
        }
    };

    template<typename T>
    bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
    {
        int cnt = 0;
        for(int row = 0;row < matrix.rows();row++)
        {
            for(int col = 0;col < matrix.cols();col++)
            {
                matrix(row,col) = vector[cnt];
                cnt++;
            }
        }
        return true;
    }

    float calcTriangleArea(cv::Point2f pts[3]);
    float calcTetragonArea(cv::Point2f pts[4]);
    double rangedAngleRad(double &angle);

    std::string symbolicToReal(std::string path);
    std::string relativeToFull(std::string relative, std::string src);
    std::string treeToPath(std::vector<std::string> &tree);
    std::string getParent(std::string path);

    std::vector<std::string> readLines(std::string file_path);
    std::vector<std::string> generatePathTree(std::string path);

    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

    Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2);
    Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);
    float calcDistance(cv::Point2f& p1, cv::Point2f& p2);

    void videoRecorder(VideoRecordParam& video_param, cv::Mat* src = nullptr);  
    bool setSharedMemory(SharedMemoryParam& shared_memory_param, int id, int image_width = 1280, int image_height = 1024);
    bool getSharedMemory(SharedMemoryParam& shared_memory_param, int id);
    bool destorySharedMemory(SharedMemoryParam& shared_memory_param);
    bool autoLabel(bool& is_init, cv::Mat &img, ofstream &file, string &path_name, double &timestamp, int &id, int &color, vector<cv::Point2f> &apex2d, cv::Point2i &roi_offset, cv::Size2i &input_size);
} // namespace global_user

#endif