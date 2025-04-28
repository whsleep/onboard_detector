/*
    FILE: uvDetector.h
    ------------------
    helper class header for uv detector
*/
#ifndef UV_DETECTOR_H
#define UV_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <math.h>
#include <vector>
#include <onboard_detector/utils.h>
#include <onboard_detector/kalmanFilter.h>
#include <queue>
#include <Eigen/Dense>

namespace onboardDetector{
    // onboardDetector 中的 UVbox 类
    class UVbox
    {
        public:
        // UVbox id
        int id; 
        // UVbox 父级id
        int toppest_parent_id; 
        // bbox
        cv::Rect bb; 

        // default constructor
        UVbox();
        // constructor for new line
        UVbox(int seg_id, int row, int left, int right);
    };


    // onboardDetector 中的 UVtracker 类
    class UVtracker
    {
        public:
        // 上次鸟瞰图的bbox
        std::vector<cv::Rect> pre_bb; 
        // 本次鸟瞰图的bbox
        std::vector<cv::Rect> now_bb; 

        // 上次鸟瞰图跟踪历史
        std::vector<vector<cv::Point2f> > pre_history; 
        // 本次鸟瞰图跟踪历史
        std::vector<vector<cv::Point2f> > now_history; 

        // 上次跟踪的卡尔曼滤波器
        std::vector<kalman_filter> pre_filter; 
        // 本次跟踪的卡尔曼滤波器
        std::vector<kalman_filter> now_filter;

        // 本次深度图检测到的bbox容器
        std::vector<cv::Rect> now_bb_D;

        // 本次深度图检测到的3Dbbox容器
        std::vector<box3D> now_box_3D;

        // 本次3Dbbox历史
        std::deque<deque<box3D>> now_box_3D_history; 
        // 上次3Dbbox历史
        std::deque<deque<box3D>> pre_box_3D_history;

        // 跟踪阈值
        float overlap_threshold; 

        // 跟踪预测速度总和，计算平均速度
        // 上次跟踪速度
        std::deque<std::deque<Eigen::MatrixXd>> pre_V;
        // 本次跟踪速度
        std::deque<std::deque<Eigen::MatrixXd>> now_V;
        
        // 计数所有识别结果中的移动次数
        std::deque<std::deque<int>> pre_count;
        std::deque<std::deque<int>> now_count;

        // 如果每个方框一直完全显示在 FOV 中，则存储其固定大小
        std::vector<box3D> fixed_box3D;


        UVtracker();

        // 读取边界框信息
        void read_bb(vector<cv::Rect> now_bb, vector<cv::Rect> now_bb_D, vector<box3D> &box_3D);

        // 检查跟踪状态
        void check_status(vector<box3D> &box_3D);

        
    };
    
    // onboardDetector 中的 UVdetector 类
    class UVdetector
    {
        public:
        // 深度图
        cv::Mat depth; 
        cv::Mat depth_show;

        // RGB图像
        cv::Mat RGB;
        // 低分辨率深度图
        cv::Mat depth_low_res; 
        // U深度图
        cv::Mat U_map; 
        // 可视化U深度图
        cv::Mat U_map_show;
        // 最小距离
        int min_dist; 
        // 最大距离
        int max_dist; 
        // 比率（深度图高度/U 图高度）
        int row_downsample; 
        // 水平方向的比例系数
        float col_scale; 
        // 感兴趣点阈值
        float threshold_point; 
        // 感兴趣线阈值
        float threshold_line; 
        // 最小长度线阈值
        int min_length_line; 
        // 是否显示3D边框
        bool show_bounding_box_U; 
        // u-depth 提取的bbox
        std::vector<cv::Rect> bounding_box_U; 
        // 鸟瞰图上的bbox
        std::vector<cv::Rect> bounding_box_B; 
        // 深度图上的bbox
        std::vector<cv::Rect> bounding_box_D; 
        // main output/topic published
        // 输出3D框
        std::vector<box3D> box3Ds; 
        std::vector<box3D> box3DsWorld;
        // vector<box3D> person_box3Ds;// 3D bboxes in world frame for persons
        
        // x,y coords of topleft corner of incoming crop from yolo
        // yolo边框左上角坐标
        int x0;
        int y0;

        //test
        int testx;
        int testy;
        int testby;
        // 相机焦距
        float fx; 
        float fy;
        // 相机主轴坐标
        float px; 
        float py;
        // 比例因子，用于将深度值从原始单位转换为实际单位
        double depthScale_; 
        // 鸟瞰图
        cv::Mat bird_view; 
        // 鸟瞰图上进行跟踪
        UVtracker tracker; 
        
        // constructor
        UVdetector();

        // 读取队列深度图
        void readdata(queue<cv::Mat> depthq);

        // 读取深度图
        void readdepth(cv::Mat depth);

        // 读取RGB
        void readrgb(cv::Mat RGB);

        // 生成u-depth
        void extract_U_map();

        // 生成U-depth图上的bbox
        void extract_bb();

        // 生成鸟瞰图
        void extract_bird_view();

        // detect
        void detect();

        // track the object
        void track();

        // output detection 
        void output();

        // display depth
        void display_depth();
        void extract_3Dbox();

        // void display_RGB();

        // display U map
        void display_U_map();

        // add tracking result to bird's view map
        void add_tracking_result();

        // display bird's view map
        void display_bird_view();

      
    };

    UVbox merge_two_UVbox(UVbox father, UVbox son);
}
#endif