/*
    FILE: uvDetector.h
    ------------------
    helper class function definitions for uv detector
*/

#include <onboard_detector/uvDetector.h>

// UVbox
namespace onboardDetector{
    UVbox::UVbox()
    {
        this->id = 0;
        this->toppest_parent_id = 0;
        // 左上,右下角构建矩形对象
        this->bb = cv::Rect(cv::Point2f(0, 0), cv::Point2f(0, 0));
    }

    UVbox::UVbox(int seg_id, int row, int left, int right)
    {
        this->id = seg_id;
        this->toppest_parent_id = seg_id;
        this->bb = cv::Rect(cv::Point2f(left, row), cv::Point2f(right, row));
    }
    // 合并两个box, 新box包含之前两个box
    UVbox merge_two_UVbox(UVbox father, UVbox son)
    {
        // merge the bounding box
        // 左上点 , y 取较小
        int top =       (father.bb.tl().y < son.bb.tl().y)?father.bb.tl().y:son.bb.tl().y;
        // 左上点 , x 取较小
        int left =      (father.bb.tl().x < son.bb.tl().x)?father.bb.tl().x:son.bb.tl().x;
        // 右下点 , y 取较大
        int bottom =    (father.bb.br().y > son.bb.br().y)?father.bb.br().y:son.bb.br().y;
        // 右下点 , x 取较大
        int right =     (father.bb.br().x > son.bb.br().x)?father.bb.br().x:son.bb.br().x;
        father.bb = cv::Rect(cv::Point2f(left, top), cv::Point2f(right, bottom));
        return father;
    }

    // UVtracker

    UVtracker::UVtracker()
    {
        // 初始化跟踪阈值
        this->overlap_threshold = 0.4;
    }
    // 读取box信息
    void UVtracker::read_bb(vector<cv::Rect> now_bb, vector<cv::Rect> now_bb_D, vector<box3D> &box_3D)
    {
        // 保存先前跟踪历史
        this->pre_history = this->now_history;
        this->now_history.clear();
        this->now_history.resize(now_bb.size());

        // 3D box history
        this->pre_box_3D_history = this->now_box_3D_history;
        this->now_box_3D_history.clear();
        this->now_box_3D_history.resize(now_bb.size());

        // kalman filters
        this->pre_filter = this->now_filter;
        this->now_filter.clear();
        this->now_filter.resize(now_bb.size());

        // sum of velocity prediction
        this->pre_V = this->now_V;
        this->now_V.clear();
        this->now_V.resize(now_bb.size());  

        // bounding box
        this->pre_bb = this->now_bb;
        this->now_bb = now_bb;
        // 深度图检测bbox
        this->now_bb_D = now_bb_D;
        // 3D检测bbox
        this->now_box_3D = box_3D;

        // 遍历3D边框双向队列
        for (size_t i=0 ; i<this->pre_box_3D_history.size() ; i++) {
            // 某一目标的历史长度超过10个
            if (this->pre_box_3D_history[i].size() > 10) {
                // 移出队头元素
                this->pre_box_3D_history[i].pop_front();
                
                // 同步删除跟踪历史头元素
                this->pre_history[i].erase(this->pre_history[i].begin());

            }
        }   
        // 固定box更新
        box_3D = this->now_box_3D;
    }
    
    void UVtracker::check_status(vector<box3D> &box_3D)
    {
        // 遍历当前鸟瞰图检测框，鸟瞰图检测框的中心点即为俯视图中跟踪障碍的中心点
        for(size_t now_id = 0; now_id < this->now_bb.size(); now_id++)
        {
            bool tracked = false;
            // 遍历上次检测到的鸟瞰图检测边框
            for(size_t pre_id = 0; pre_id < this->pre_bb.size(); pre_id++)
            {
                // 计算两个方框的交集
                cv::Rect overlap = this->now_bb[now_id] & this->pre_bb[pre_id];
                // 计算now_bb和pre_bb两方框中心点的距离
                float dist = std::sqrt( std::pow((this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width)-(this->pre_bb[pre_id].x + 0.5 * this->pre_bb[pre_id].width),2) + std::pow((this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height-(this->pre_bb[pre_id].y + 0.5 * this->pre_bb[pre_id].height)),2) );
                // 可以发生重叠的最大距离
                float metric = std::sqrt( std::pow(this->now_bb[now_id].width+this->pre_bb[pre_id].width,2) + std::pow(this->now_bb[now_id].height+this->pre_bb[pre_id].height,2) )/2;
                // 重叠面积/最大原始框面积 大于 跟踪阈值 或者 两边框发送重叠
                if(max(overlap.area() / float(this->now_bb[now_id].area()), overlap.area() / float(this->pre_bb[pre_id].area())) >= this->overlap_threshold || dist<=metric)
                {
                    // 匹配成功
                    tracked = true;
                    
                    // 记录上次历史数据的匹配点信息
                    this->now_history[now_id] = this->pre_history[pre_id];
                    // 之后添加当前点
                    this->now_history[now_id].push_back(cv::Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
                    // 同时记录3D框的匹配点
                    this->now_box_3D_history[now_id] = this->pre_box_3D_history[pre_id];
                    
                    // 深度图的检测结果不在边框范围内，才进行更新
                    if (this->now_bb_D[now_id].tl().x>5 && this->now_bb_D[now_id].tl().y>5 && this->now_bb_D[now_id].br().x<635 && this->now_bb_D[now_id].br().y<475) {
                        this->now_box_3D_history[now_id].push_back(this->now_box_3D[now_id]);
                    } 
                    // 记录匹配到的速度信息
                    this->now_V[now_id] = this->pre_V[pre_id];
                    // 记录匹配的滤波器
                    this->now_filter[now_id] = this->pre_filter[pre_id];

                    break;
                } 

            }
            // 匹配失败
            if(!tracked)    
            {
                // std::cout<<now_id<<" LOSS TRACK\n"<<std::endl;
                // add current detection to history
                // 继续记录历史，但不再更新
                this->now_history[now_id].push_back(cv::Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
                // 设定状态静止
                Eigen::MatrixXd V(2,1);
                V << 0.0, 0.0;
                this->now_V[now_id].push_back(V);
                // 深度图的检测结果不在边框范围内，才进行更新
                if (this->now_bb_D[now_id].tl().x>5 && this->now_bb_D[now_id].tl().y>5 && this->now_bb_D[now_id].br().x<635 && this->now_bb_D[now_id].br().y<475) {
                    this->now_box_3D_history[now_id].push_back(this->now_box_3D[now_id]);
                }

            }
        }
    }  
    // UVdetector     
    // 构造函数
    UVdetector::UVdetector()
    {
        // 深度图高度/U-depth盖度 = 4
        this->row_downsample = 4;
        this->col_scale = 0.5;
        this->min_dist = 10;
        this->max_dist = 8000;// unit: m
        this->threshold_point = 3;
        this->threshold_line = 2;
        this->min_length_line = 6;
        this->show_bounding_box_U = true;
        // the following intrinsic parameters can be found in /camera/../camera_info
        this->fx = 608.08740234375;
        this->fy = 608.1791381835938;
        this->px = 317.48284912109375;
        this->py = 234.11557006835938;

        this->x0 = 0;
        this->y0 = 0;
    }

    void UVdetector::readdata(queue<cv::Mat> depthq)
    {
        // 取队列头尾矩阵每个元素的最大值
        this->depth = max(depthq.front(), depthq.back());
        // 
        double minVal; 
        double maxVal; 
        cv::Point minLoc; 
        cv::Point maxLoc;
        // 找到矩阵中的最小值和最大值及其位置
        minMaxLoc( this->depth, &minVal, &maxVal, &minLoc, &maxLoc );   
    }

    void UVdetector::readdepth(cv::Mat depth){
        this->depth = depth;
        double minVal; 
        double maxVal; 
        cv::Point minLoc; 
        cv::Point maxLoc;
        // 找到矩阵中的最小值和最大值及其位置
        minMaxLoc( this->depth, &minVal, &maxVal, &minLoc, &maxLoc ); 
    }

    void UVdetector::readrgb(cv::Mat RGB)
    {
        this->RGB = RGB;
        // 前后尺寸不同会进行缩放
        resize(this->RGB, this->RGB, cv::Size(720,400));
        // imshow("RGB", this->RGB);
    }

    void UVdetector::extract_U_map()
    {
        // rescale depth map
        cv::Mat depth_rescale;
        // 输入矩阵,输出矩阵,空尺寸(根据缩放因子自动调节尺寸),列方向缩放因子,行方向缩放因子
        resize(this->depth, depth_rescale, cv::Size(),this->col_scale , 1);
        // 创建全零矩阵,用于存储第分辨率图
        cv::Mat depth_low_res_temp = cv::Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);
        // 计算 u-depth 高度
        uint8_t histSize = this->depth.rows / this->row_downsample;
        // 深度信息映射到u-depth上
        int bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
        // 创建一个全零矩阵 this->U_map，用于存储 U-map
        this->U_map = cv::Mat::zeros(histSize, depth_rescale.cols, CV_8UC1);
        
        int depth_rescale_val = 0;

        for(int col = 0; col < depth_rescale.cols; col++)
        {
            for(int row = 0; row < depth_rescale.rows; row++)
            {
                // printf("HERE %d,%d",row,col);
                // 深度值转换为实际单位
                depth_rescale_val = int((float(depth_rescale.at<unsigned short>(row, col))/this->depthScale_)*1000.0);
                // printf("raw depth %d, %d: %d\n",row,col,depth_rescale_val);
                // 在可接受范围内
                if(depth_rescale_val > this->min_dist && depth_rescale_val < this->max_dist)
                {
                    // h * (val - min) / (max - min) ,深度信息映射到u-depth上 
                    uint8_t bin_index = (depth_rescale_val - this->min_dist) / bin_width;
                    depth_low_res_temp.at<uchar>(row, col) = bin_index;
                    // printf("depth val %d",depth_rescale_val);
                    // 
                    if(this->U_map.at<uchar>(bin_index, col) < 255)
                    {
                        // printf("here %d",this->U_map.at<uchar>(bin_index, col));
                        this->U_map.at<uchar>(bin_index, col) ++;
                    }
                }
            }
        }
        this->depth_low_res = depth_low_res_temp;

        // smooth the U map
        
        GaussianBlur(this->U_map, this->U_map, cv::Size(5,9), 10, 10);
        // printf("rescaled depth map: %d", depth_rescale.at<unsigned short>(testy,testx));
    }

    void UVdetector::extract_bb()
    {
        // 创建mask用于存储u-depth像素ID
        std::vector<vector<int> > mask(this->U_map.rows, vector<int>(this->U_map.cols, 0));
        // 用于判断是否为兴趣点的阈值
        int u_min = this->threshold_point * this->row_downsample;
        // 当前行像素值总和
        int sum_line = 0;
        // 当前行的最大像素
        int max_line = 0;
        // 当前行的长度
        int length_line = 0;
        // 当前分割ID,从1开始
        int seg_id = 0;
        // 存储检测到的边界框
        std::vector<UVbox> UVboxes;
        // printf("Umap rows %d\n",this->U_map.rows);
        // 遍历U-depth
        for(int row = 0; row < this->U_map.rows; row++)
        {
            // 列遍历
            for(int col = 0; col < this->U_map.cols; col++)
            {
                // 判断是否为兴趣点
                if(this->U_map.at<uchar>(row,col) >= u_min) // num of points at this depth >= u_min
                {
                    length_line++;
                    sum_line += this->U_map.at<uchar>(row,col);
                    max_line = (this->U_map.at<uchar>(row,col) > max_line)?this->U_map.at<uchar>(row,col):max_line;
                }
                // 非兴趣点 或者 遍历到倒数第二行
                if(this->U_map.at<uchar>(row,col) < u_min || col == this->U_map.cols - 1)
                {
                    // 如果已经到倒数第二列,不再进行遍历
                    col = (col == this->U_map.cols - 1)? col + 1:col;
                    // 满足要求的线段
                    if(length_line > this->min_length_line && sum_line > this->threshold_line * max_line)
                    {
                        // 分割ID++
                        seg_id++;
                        // 加入uvbox容器
                        UVboxes.push_back(UVbox(seg_id, row, col - length_line, col - 1));
                        // 将线段覆盖的像素点赋予分割编号(也是UVboxes索引)
                        for(int c = col - length_line; c < col - 1; c++)
                        {
                            mask[row][c] = seg_id;
                        }
                        // 当当前行不是第一行时，我们需要合并邻行分割
                        if(row != 0)
                        {
                            // merge all parents
                            for(int c = col - length_line; c < col - 1; c++)
                            {
                                // 上一行的像素编号不为0
                                if(mask[row - 1][c] != 0)
                                {
                                    // 上一行像素的UVbox最高父ID 小于 当前最新 UVbox ID，即上一行优先级高
                                    if(UVboxes[mask[row - 1][c] - 1].toppest_parent_id < UVboxes.back().toppest_parent_id)
                                    {
                                        // 合并到上一行
                                        UVboxes.back().toppest_parent_id = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                    }
                                    // 当前行的优先级更高
                                    else
                                    {
                                        // 获取上一行的编号
                                        int temp = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                        // 遍历UVbox
                                        for(size_t b = 0; b < UVboxes.size(); b++)
                                        {
                                            // UVBox中所有编号为temp的复选框id均设置为当前UVbox的id，因为当前优先级高于上一次，需要把和上一次相同编号的box全部合并
                                            UVboxes[b].toppest_parent_id = (UVboxes[b].toppest_parent_id == temp)?UVboxes.back().toppest_parent_id:UVboxes[b].toppest_parent_id;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // 列遍历到非兴趣点 或者 到列末尾, 拟合信息进行刷新,准备记录下一次列遍历信息
                    sum_line = 0;
                    max_line = 0;
                    length_line = 0;
                }
            }
        }
        
        // 清空U图检测到的box
        this->bounding_box_U.clear();
        // 合并相同父级的box
        for(size_t b = 0; b < UVboxes.size(); b++)
        {
            // 只有局部区域最高级别的box id和toppest_parent_id相同，级别低的toppest_parent_id会被更改
            if(UVboxes[b].id == UVboxes[b].toppest_parent_id)
            {
                // 遍历之后的， 先入box的级别更高，所以s之前的级别均比s高，所以无法进行合并
                for(size_t s = b + 1; s < UVboxes.size(); s++)
                {
                    if(UVboxes[s].toppest_parent_id == UVboxes[b].id)
                    {
                        UVboxes[b] = merge_two_UVbox(UVboxes[b], UVboxes[s]);

                    }
                }
                // 面积大于25 
                if(UVboxes[b].bb.area() >= 25)
                {
                    this->bounding_box_U.push_back(UVboxes[b].bb);
                    // printf("bbox_U [b] %f",UVboxes[b].bb.tl().y);
                }
            }
        }
    }

    void UVdetector::detect()
    {
        // extract U map from depth
        this->extract_U_map();

        // extract bounding box from U map
        this->extract_bb();

        // extract bounding box
        this->extract_bird_view();

        // extract object's height
        // this->extract_height();
    }  

    void UVdetector::display_depth()
    {
        // 深度图像素映射到(0-255)
        cv::Mat depth_normalized;
        this->depth.copyTo(depth_normalized);
        // 找到 depth_normalized 中的 min,max
        double min, max;
        cv::minMaxIdx(depth_normalized, &min, &max);
        // 函数将深度图的值按比例缩放到 0 到 255 的范围。缩放因子为 255. / max
        cv::convertScaleAbs(depth_normalized, depth_normalized, 255. / max);
        depth_normalized.convertTo(depth_normalized, CV_8UC1);
        // 函数将灰度图像转换为彩色图像
        applyColorMap(depth_normalized, depth_normalized, cv::COLORMAP_BONE);

        // loop for adding bounding boxes
        // 绘制边界框
        for (size_t i=0;i<this->bounding_box_D.size();i++){
            rectangle(depth_normalized, bounding_box_D[i], cv::Scalar(0, 255, 0), 5, 8, 0);
        }
        this->depth_show = depth_normalized;
        // imshow("Depth", depth_normalized);
        // waitKey(1);
    }

    void UVdetector::extract_3Dbox()
    {   
        // 复制深度图
        cv::Mat depth_resize;
        resize(depth, depth_resize, cv::Size(), this->col_scale, 1);
        // 获取U图高度
        float histSize = this->depth.rows / this->row_downsample;
        // 获取U图宽度
        float bin_width = ceil((this->max_dist - this->min_dist) / histSize);
        // 竖直方向检测参数
        int x;
        int y_up;
        int y_down;
        int width;
        int bin_index_small;
        int bin_index_large;
        float depth_in_near;
        float depth_of_depth;
        float depth_in_far;
        float depth_resize_val;

        size_t im_frame_x;
        size_t im_frame_x_width;
        size_t im_frame_y;
        size_t im_frame_y_width;

        // GaussianBlur(depth_resize, depth_resize, cv::Size(5,9), 0, 0);

        // parameter for tunning
        int num_check  = 15;

        // 清空3Dbox信息
        this->box3Ds.clear();
        this->bounding_box_D.clear();
        // 根据U-depth生成的bounding_box_U，计算深度图上的h
        for (size_t b = 0; b < this->bounding_box_U.size(); b++) {
            // U图边框左上顶点x坐标
            x = this->bounding_box_U[b].tl().x;
            // U图边框宽度
            width = this->bounding_box_U[b].width;
            // 获取深度图y坐标最大值
            y_up = depth_resize.rows;
            // 深度图y坐标最小值
            y_down = 0;
            // U图y坐标范围 
            bin_index_small = this->bounding_box_U[b].tl().y;
            bin_index_large = this->bounding_box_U[b].br().y;
            // U图y坐标还原到深度图范围
            depth_in_near = (bin_index_small * bin_width + this->min_dist);
            depth_of_depth = (bin_index_large - bin_index_small) * bin_width;
            depth_in_far = depth_of_depth*1.3 + depth_in_near; 
            // 竖向连续检查(在深度允许范围内并且满足相邻连续要求)
            for (int i = x ; i < x + width; i++) { // for several middle coloums
                for (int j = 0; j < depth_resize.rows - 1; j++) { // for each row
                    // 获取深度像素点的实际距离
                    depth_resize_val = (float(depth_resize.at<unsigned short>(j,i))/this->depthScale_)*1000.0;
                    // 在允许的深度范围内
                    if (float(depth_resize_val) >= depth_in_near && depth_resize_val <= depth_in_far) {
                        // 相同列上检测更多点
                        for (int check = 0; check < num_check; check++) { // check some more points in the coloum
                            depth_resize_val = (float(depth_resize.at<unsigned short>(j + check + 1,i))/this->depthScale_)*1000.0;
                            // 检测点不在深度允许范围内
                            if (depth_resize_val < depth_in_near || 
                            depth_resize_val > depth_in_far) {
                                // 跳出当前循环
                                break;
                            }
                            // 所有检测点均符合要求
                            if (check == num_check-1) {
                                // 拓长列检测范围
                                if (y_up > j) y_up = j;
                                if (y_down < j) y_down = j;
                            }
                        }
                    }
                }
            }
            // 检测完成后获取U图2D边框上对应深度图 的边框高度信息，深度图和U图的边框宽度相等
            // 根据缩放因子扩大到原depth图上
            // U图x坐标还原到深度图上
            float bb_x = x / this->col_scale;
            // U图2D边框宽度还原到深度图宽度上
            float bb_width = width / this->col_scale;
            // 深度图边框y坐标即为检测到的y_up
            float bb_y = y_up;
            // 高度即为y_down-y_up
            float bb_height = y_down-y_up;
            // 由U图2D边框转换到深度图的2D边框
            this->bounding_box_D.push_back(cv::Rect(bb_x, bb_y, bb_width, bb_height));

            // 计算3Dbox参数
            box3D curr_box;
            // 计算3Dbox中心点位置
            im_frame_x  = (x + width / 2) / this->col_scale;
            im_frame_x_width = width / this->col_scale;
            
            // 三维的y即为深度值
            // 获取3Dbox深度中心坐标
            int Y_w = (depth_in_near + depth_in_far) / 2;
            // 获取3Dbox中心y坐标
            im_frame_y = (y_down + y_up) / 2;
            im_frame_y_width = y_down - y_up;
            
            testx = im_frame_x;
            testy = im_frame_y;
            testby = bin_index_small;

            // 获取相机坐标下3Dbox的信息
            curr_box.x =  (im_frame_x-this->px)*Y_w/this->fx;
            curr_box.y = (im_frame_y-this->py)*Y_w/this->fy;
            curr_box.x_width = im_frame_x_width*Y_w/this->fx;
            curr_box.y_width = im_frame_y_width*Y_w/this->fy;
            curr_box.z = Y_w;
            curr_box.z_width = depth_in_far-depth_in_near; 

            // convert from mm to m
            curr_box.x /=1000.0;
            curr_box.y /=1000.0;
            curr_box.z /=1000.0;
            curr_box.x_width /=1000.0;
            curr_box.y_width /=1000.0;
            curr_box.z_width /=1000.0;
            // std::cout<<"uv box on came raw z_width "<<curr_box.z_width<<std::endl;
            box3Ds.push_back(curr_box);
            // printf("3d box %d on cam: %f, %f, %f,%f ,%f, %f\n",curr_box.x,curr_box.y,curr_box.z,curr_box.x_width,curr_box.y_width,curr_box.z_width);
            // printf("depth in near: %f \n",depth_in_near);
        }
    }
        
    void UVdetector::display_U_map()
    {
        // visualize with bounding box
        if(this->show_bounding_box_U)
        {
            this->U_map = this->U_map * 10;
            this->U_map_show = this->U_map;

            double min, max;
            // 获取U_map_show中的最大最小值
            cv::minMaxIdx(this->U_map_show, &min, &max);
            // 将U_map_show转化为RGB图
            cvtColor(this->U_map_show, this->U_map_show, cv::COLOR_GRAY2RGB);
            // 将图像缩放到[0,255]并转化为8位无符号型数据
            cv::convertScaleAbs(this->U_map_show, this->U_map_show, 255./ max);
            this->U_map_show.convertTo(this->U_map_show, CV_8UC1);
            // 将U_map_show映射位 jet 色阶
            applyColorMap(this->U_map_show, this->U_map_show, cv::COLORMAP_JET);
            // 绘制U图上的2D边框
            for(size_t b = 0; b < this->bounding_box_U.size(); b++)
            {
                cv::Rect final_bb = cv::Rect(this->bounding_box_U[b].tl(),cv::Size(this->bounding_box_U[b].width, 2 * this->bounding_box_U[b].height));
                rectangle(this->U_map_show, final_bb, cv::Scalar(0, 255, 0), 1, 8, 0);
                
                // circle(this->U_map_show, cv::Point2f(this->bounding_box_U[b].tl().x + 0.5 * this->bounding_box_U[b].width, this->bounding_box_U[b].br().y ), 2, cv::Scalar(0, 0, 255), 5, 8, 0);
            }
        } 
        // imshow("U map", this->U_map_show);
        // waitKey(1);
    }

    // unit is 10 mm?
    // x, y is bottome left point
    void UVdetector::extract_bird_view()
    {
        // 
        uint8_t histSize = this->depth.rows / this->row_downsample;
        uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
        this->bounding_box_B.clear();
        this->bounding_box_B.resize(this->bounding_box_U.size());

        for(size_t b = 0; b < this->bounding_box_U.size(); b++)
        {
            // U_map bounding box -> Bird's view bounding box conversion
            // U图的行索引转对应的深度信息
            float bb_depth = this->bounding_box_U[b].br().y * bin_width / 10;
            // 鸟瞰图的宽和高
            float bb_width = bb_depth * this->bounding_box_U[b].width / this->fx;
            float bb_height = this->bounding_box_U[b].height * bin_width / 10;
            // 鸟瞰图的左上角坐标
            float bb_x = bb_depth * (this->bounding_box_U[b].tl().x / this->col_scale - this->px) / this->fx;
            float bb_y = bb_depth - 0.5 * bb_height;// assume farthest depth value is the depth of center point, then assume detected depth difference is the depth of the whole body. y is the depth direction
            this->bounding_box_B[b] = cv::Rect(bb_x, bb_y, bb_width, bb_height);
        }

        // initialize the bird's view
        this->bird_view = cv::Mat::zeros(500, 1000, CV_8UC1);
        cvtColor(this->bird_view, this->bird_view, cv::COLOR_GRAY2RGB);
    }

    void UVdetector::display_bird_view() 
    {
        // center poin
        // 相机在鸟瞰图中的位置
        cv::Point2f center = cv::Point2f(this->bird_view.cols / 2, this->bird_view.rows);
        // 鸟瞰图左边界点
        cv::Point2f left_end_to_center = cv::Point2f( this->bird_view.rows * (0 - this->px) / this->fx, -this->bird_view.rows);
        // 鸟瞰图右边界点
        cv::Point2f right_end_to_center = cv::Point2f( this->bird_view.rows * (this->depth.cols - this->px) / this->fx, -this->bird_view.rows);

        // draw the two side lines
        // 绘制鸟瞰图视野范围
        line(this->bird_view, center, center + left_end_to_center, cv::Scalar(0, 255, 0), 3, 8, 0);
        line(this->bird_view, center, center + right_end_to_center, cv::Scalar(0, 255, 0), 3, 8, 0);

        // 绘制鸟瞰图的2D边框
        for(size_t b = 0; b < this->bounding_box_U.size(); b++)
        {
            // change coordinates
            cv::Rect final_bb = this->bounding_box_B[b];
            final_bb.y = center.y - final_bb.y - final_bb.height;
            final_bb.x = final_bb.x + center.x; 
            // draw center 
            cv::Point2f bb_center = cv::Point2f(final_bb.x + 0.5 * final_bb.width, final_bb.y + 0.5 * final_bb.height);
            rectangle(this->bird_view, final_bb, cv::Scalar(0, 0, 255), 3, 8, 0);
            // 绘制鸟瞰图边框中心点
            circle(this->bird_view, bb_center, 3, cv::Scalar(0, 0, 255), 5, 8, 0);
        }

        // show
        resize(this->bird_view, this->bird_view, cv::Size(), 0.5, 0.5);
        // imshow("Bird's View", this->bird_view);
        // waitKey(1);
    }

    void UVdetector::add_tracking_result()
    {
        // 确定俯视中心
        cv::Point2f center = cv::Point2f(this->bird_view.cols / 2, this->bird_view.rows);
        // 遍历匹配本次鸟瞰图复选框
        for(size_t b = 0; b < this->tracker.now_bb.size(); b++)
        {
            // box中心坐标变换到俯视中心坐标下
            cv::Point2f estimated_center = cv::Point2f(this->tracker.now_filter[b].output(0), this->tracker.now_filter[b].output(1));
            estimated_center.y = center.y - estimated_center.y;
            estimated_center.x = estimated_center.x + center.x; 
            // 绘制复选框中心
            circle(this->bird_view, estimated_center, 5, cv::Scalar(0, 255, 0), 5, 8, 0);
            // 记录加速度
            cv::Point2f bb_size = cv::Point2f(this->tracker.now_filter[b].output(4), this->tracker.now_filter[b].output(5));
            rectangle(this->bird_view, cv::Rect(estimated_center - 0.5 * bb_size, estimated_center + 0.5 * bb_size), cv::Scalar(0, 255, 0), 3, 8, 0);
            // 记录速度
            cv::Point2f velocity = cv::Point2f(this->tracker.now_filter[b].output(2), this->tracker.now_filter[b].output(3));
            velocity.y = -velocity.y;// y direction in birdvie map is in opposite of opencv::line default
            // printf("velocity in birdview 10mm/s: %f, %f , center x ,y: %f, %f, bbox size x, y:%f, %f\n", velocity.x, -velocity.y, estimated_center.x, estimated_center.y, bb_size.x, bb_size.y);
            // 绘制速度方向
            line(this->bird_view, estimated_center, estimated_center + velocity, cv::Scalar(255, 255, 255), 3, 8, 0);
            // 绘制本次跟踪历史
            for(size_t h = 1; h < this->tracker.now_history[b].size(); h++)
            {
                // trajectory
                cv::Point2f start = this->tracker.now_history[b][h - 1];
                start.y = center.y - start.y;
                start.x = start.x + center.x;
                cv::Point2f end = this->tracker.now_history[b][h];
                end.y = center.y - end.y;
                end.x = end.x + center.x;
                line(this->bird_view, start, end, cv::Scalar(0, 0, 255), 3, 8, 0);
            }
        }
    }
    
    void UVdetector::track()
    {
        // float before = this->box3Ds[0].x_width;

        this->tracker.read_bb(this->bounding_box_B, this->bounding_box_D, this->box3Ds);
        this->tracker.check_status(this->box3Ds);
        this->add_tracking_result();
        // float after = this->box3Ds[0].x_width;
        // printf("verify : %f, %f", before, after);
    }
}