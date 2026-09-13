#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <costmap_2d/cost_values.h>   // 必须加上这一行

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
class LaserToCostmap
{
public:
    LaserToCostmap()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh("~");

        // 订阅激光话题
        scan_sub_ = nh.subscribe("/go2/laser/scan", 1, &LaserToCostmap::scanCallback, this);
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);

        // costmap 初始化参数
        double resolution = 0.05;   // 每个栅格0.05m
        unsigned int cells_x = 200; // 10m(定义了 Costmap（代价地图的尺寸，地图覆盖的实际范围)
        unsigned int cells_y = 200; // 10m
        //地图左下角在世界坐标 (-5, -5):
        costmap_.reset(new costmap_2d::Costmap2D(cells_x, cells_y, resolution, -5.0, -5.0, 0)); // 中心在机器人附近
        costmap_->setDefaultValue(costmap_2d::FREE_SPACE);

        edt_pub_ = nh.advertise<sensor_msgs::Image>("edt_map", 1);
        ROS_INFO("LaserToCostmap initialized. Waiting for /go2/laser/scan...");
    }

private:
    ros::Subscriber scan_sub_;
    ros::Publisher map_pub_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<costmap_2d::Costmap2D> costmap_;

    cv::Mat edt_map_;  // 存储欧氏距离变换结果
    std::mutex edt_mutex_;  // 保护EDT地图的线程安全

    ros::Publisher edt_pub_;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // 转换 LaserScan -> PointCloud2
        sensor_msgs::PointCloud2 cloud;
        try
        {
            projector_.transformLaserScanToPointCloud("trunk", *scan_msg, cloud, tf_buffer_);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF transform failed: %s", ex.what());
            return;
        }

        // 更新 costmap
        updateCostmap(cloud);

        // 发布 OccupancyGrid 以便 RViz 查看
        publishOccupancyGrid();
        publishEDTMap();
    }

    void updateCostmap(const sensor_msgs::PointCloud2& cloud)
    {
        // 清空旧地图
        for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); ++i)
            {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); ++j)
                    {
                    costmap_->setCost(i, j, costmap_2d::FREE_SPACE);
                    }
            }
        // 遍历点云，过滤过近的点（如距离小于0.3m的点视为自身结构）
        const double MIN_OBSTACLE_DISTANCE = 0.2; // 可根据机器人尺寸调整(0.15就不行了)
        const double ROBOT_RADIUS = 0.2;  // 新增：机器人半径
        // 遍历点云，将点投影到 costmap
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"), iter_y(cloud, "y");
             iter_x != iter_x.end(); ++iter_x, ++iter_y)
        {
            double wx = *iter_x;
            double wy = *iter_y;

            // 过滤距离机器人过近的点（自身结构）
            if (std::hypot(wx, wy) < MIN_OBSTACLE_DISTANCE) {
                continue; // 跳过自身结构点
            }
            unsigned int mx, my;

            if (costmap_->worldToMap(wx, wy, mx, my))//worldToMap是costmap_2d提供的坐标转换函数,把世界坐标转成栅格下标
            {//zhi you 激光点(障碍)cai neng zhuan huan cheng gong(fan hui true)
                costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);

                // 新增：根据机器人半径膨胀障碍（转换半径为栅格数）
                int radius_cells = static_cast<int>(std::ceil(ROBOT_RADIUS / costmap_->getResolution()));
                inflateCell(mx, my, radius_cells);  // 用机器人半径进行第一次膨胀
            }
        }

        
        // 简单的膨胀（inflation）
        const int inflation_radius = 4; // 单位：cell
        for (unsigned int mx = 0; mx < costmap_->getSizeInCellsX(); ++mx)
        {
            for (unsigned int my = 0; my < costmap_->getSizeInCellsY(); ++my)
            {
                if (costmap_->getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE)
                {//getCost是costmap_2d官方库函数,不会触发任何计算，只是从内部 unsigned char* costmap_ 数组里按索引取一个字节
                    inflateCell(mx, my, inflation_radius);
                }
            }
        }
        // 完成障碍物标记和膨胀后计算EDT
        computeEDT();

    }

    void inflateCell(unsigned int mx, unsigned int my, int radius)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                int nx = mx + dx;
                int ny = my + dy;
                if (nx < 0 || ny < 0 || nx >= (int)costmap_->getSizeInCellsX() || ny >= (int)costmap_->getSizeInCellsY())
                    continue;

                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= radius)
                {
                    unsigned char new_cost = 254 - static_cast<unsigned char>((dist / radius) * 200);
                    unsigned char old_cost = costmap_->getCost(nx, ny);
                    if (new_cost > old_cost)
                        costmap_->setCost(nx, ny, new_cost);
                }
            }
        }
    }

    void publishOccupancyGrid()
    {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "trunk";
        grid.info.resolution = costmap_->getResolution();
        grid.info.width = costmap_->getSizeInCellsX();
        grid.info.height = costmap_->getSizeInCellsY();
        grid.info.origin.position.x = costmap_->getOriginX();
        grid.info.origin.position.y = costmap_->getOriginY();
        grid.info.origin.position.z = 0.0;

        grid.data.resize(grid.info.width * grid.info.height);
        //std::cout << "地图栅格的宽度grid.info.width=" << grid.info.width << std::endl;(da yin chu lai de zhi wei 200)
        //std::cout << "地图栅格的grid.info.origin.position.x=" << grid.info.origin.position.x << std::endl;(da yin chu lai de zhi wei -5)
        for (unsigned int y = 0; y < grid.info.height; ++y)
        {
            for (unsigned int x = 0; x < grid.info.width; ++x)
            {
                unsigned char cost = costmap_->getCost(x, y);
                if (cost == costmap_2d::LETHAL_OBSTACLE)
                    grid.data[y * grid.info.width + x] = 100;
                else if (cost == costmap_2d::FREE_SPACE)
                    grid.data[y * grid.info.width + x] = 0;
                else
                    grid.data[y * grid.info.width + x] = (int)((cost / 254.0) * 100);
            }
        }

        map_pub_.publish(grid);
    }
    void computeEDT()
    {
        // 将costmap转换为OpenCV二值图像（障碍为0，自由空间为255）
        cv::Mat binary_map(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_8UC1);
        
        for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); ++y)
        {
            for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); ++x)
            {
                unsigned char cost = costmap_->getCost(x, y);
                // 障碍区域设为0，自由空间设为255
                binary_map.at<uchar>(y, x) = (cost == costmap_2d::LETHAL_OBSTACLE) ? 0 : 255;
            }
        }

        // 计算欧氏距离变换
        edt_map_.create(binary_map.size(), CV_32FC1);
        cv::distanceTransform(binary_map, edt_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        // 距离单位转换（从栅格数转为米）
        edt_map_ *= costmap_->getResolution();
        
        std::lock_guard<std::mutex> lock(edt_mutex_);
    }

    // 添加发布EDT地图的函数（可选，用于可视化）
    void publishEDTMap()
    {
        std::lock_guard<std::mutex> lock(edt_mutex_);
        if (edt_map_.empty()) return;

        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "trunk";
        msg.height = edt_map_.rows;
        msg.width = edt_map_.cols;
        msg.encoding = "32FC1";
        msg.is_bigendian = false;
        msg.step = edt_map_.cols * sizeof(float);
        msg.data.assign((uchar*)edt_map_.data, (uchar*)edt_map_.data + edt_map_.total() * edt_map_.elemSize());
        
        edt_pub_.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_costmap");//rqt cha dao de hua ti ming: /laser_to_costmap/costmap
    LaserToCostmap node;
    ros::spin();
    return 0;
}

