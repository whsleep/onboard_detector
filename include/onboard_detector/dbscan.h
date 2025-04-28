/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

// 未分类点 - 该点尚未被分配到任何聚类中
#define UNCLASSIFIED -1
// 核心点 - 表示该点在其eps邻域内至少包含minPts个点
#define CORE_POINT 1
// 边界点 - 表示该点在其eps邻域内的点数少于minPts，但位于某个核心点的邻域内
#define BORDER_POINT 2
// 噪声点
#define NOISE -2
/* 结果宏 */
// 操作成功
#define SUCCESS 0
// 操作失败
#define FAILURE -3

using namespace std;
namespace onboardDetector{
    // onboardDetector命名空间下的三维数据结构体
    typedef struct Point_
    {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    }Point;
    // DBSCAN类
    class DBSCAN {
    public:    
        // 构造函数( 最小点数 , 邻域半径 , 点集 )
        DBSCAN(unsigned int minPts, float eps, vector<Point> points){
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();
        }
        // 析构函数
        ~DBSCAN(){}

        // 执行聚类函数
        int run();
        vector<int> calculateCluster(Point point);
        int expandCluster(Point point, int clusterID);
        inline double calculateDistance(const Point& pointCore, const Point& pointTarget);

        int getTotalPointSize() {return m_pointSize;}
        int getMinimumClusterSize() {return m_minPoints;}
        int getEpsilonSize() {return m_epsilon;}
        
    public:
        vector<Point> m_points;
        
    private:    
        // 点集大小
        unsigned int m_pointSize;
        // 聚类的最小点数
        unsigned int m_minPoints;
        // 邻域半径
        float m_epsilon;
    };
}
#endif // DBSCAN_H
