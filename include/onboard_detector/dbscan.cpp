/*
    FILE: dbscan.h
    ------------------
    helper class function definitions for dbscan
*/
#include <onboard_detector/dbscan.h>
#include <iostream>

namespace onboardDetector{
    int DBSCAN::run()
    {
        // 初始化点云簇编号
        int clusterID = 1;
        vector<Point>::iterator iter;
        for(iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            // 仅对未分类点云进行点云簇计算
            if ( iter->clusterID == UNCLASSIFIED )
            {
                // 拓展计算
                if ( expandCluster(*iter, clusterID) != FAILURE )
                {
                    // 如果拓展成功,点云簇编号自增
                    clusterID += 1;
                }
            }
        }
        return 0;
    }
    // 拓展点云簇
    int DBSCAN::expandCluster(Point point, int clusterID)
    {    
        // 获取该点云周围直接密度可达点
        vector<int> clusterSeeds = calculateCluster(point);
        // 判断该点云是否构成核心点
        if ( clusterSeeds.size() < m_minPoints )
        {
            // 标记未噪声点
            point.clusterID = NOISE;
            // 该点云无法构成点云簇
            return FAILURE;
        }
        else
        {
            /* 当前计算点云簇的核心点索引 indexCorePoint */
            int index = 0, indexCorePoint = 0;
            // 初始化迭代器,遍历该核心点的直接密度可达点
            vector<int>::iterator iterSeeds;
            for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
            {
                // 该核心点周围的直接密度可达点标记点云簇编号
                m_points.at(*iterSeeds).clusterID = clusterID;
                if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
                {
                    // 记录核心点索引
                    indexCorePoint = index;
                }
                ++index;
            }
            // 清除核心点索引
            clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);
            // 遍历该核心点周围的直接密度可达点(不包含核心点)
            for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
            {
                // 计算该核心点的密度可达点
                vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
                // 判断是否构成核心点
                if ( clusterNeighors.size() >= m_minPoints )
                {
                    // 遍历计算该核心点的直接密度可达点的直接密度可达点集合
                    vector<int>::iterator iterNeighors;
                    for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                    {
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                        {
                            // 该点不为噪声点,且为未分类点
                            if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                            {
                                // 将密度可达点存入点云簇
                                clusterSeeds.push_back(*iterNeighors);
                                // 记录点云簇大小
                                n = clusterSeeds.size();
                            }
                            // 密度可达点绑定点云簇编号
                            m_points.at(*iterNeighors).clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;
        }
    }
    // 计算直接密度可达点
    vector<int> DBSCAN::calculateCluster(Point point)
    {
        // 起始索引
        int index = 0;
        // 创建Point迭代器遍历所有点云
        vector<Point>::iterator iter;
        // 点云簇索引容器
        vector<int> clusterIndex;
        for( iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            // 该点云与所有点云距离(这里包含核心点自身)
            if ( calculateDistance(point, *iter) <= m_epsilon )
            {
                // 记录该点云周围的直接密度可达点索引
                clusterIndex.push_back(index);
            }
            index++;
        }
        // 返回该点云的直接密度可达点索引
        return clusterIndex;
    }
    // 内联函数快速调用
    inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
    {
        return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
    }
}


