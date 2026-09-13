#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <costmap_2d/cost_values.h>   // Required for the cost constants below.

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

        // Subscribe to the laser-scan topic.
        scan_sub_ = nh.subscribe("/go2/laser/scan", 1, &LaserToCostmap::scanCallback, this);
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);

        // Initialize the costmap parameters.
        double resolution = 0.05;   // Each grid cell is 0.05 m.
        unsigned int cells_x = 200; // A 200-cell grid covers 10 m at this resolution.
        unsigned int cells_y = 200; // 10m
        // Place the lower-left corner at world coordinates (-5, -5).
        costmap_.reset(new costmap_2d::Costmap2D(cells_x, cells_y, resolution, -5.0, -5.0, 0)); // Keep the robot near the map center.
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

    cv::Mat edt_map_;  // Stores the Euclidean distance transform.
    std::mutex edt_mutex_;  // Protects concurrent access to the EDT map.

    ros::Publisher edt_pub_;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // Convert LaserScan to PointCloud2.
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

        // Update the costmap.
        updateCostmap(cloud);

        // Publish the OccupancyGrid for visualization in RViz.
        publishOccupancyGrid();
        publishEDTMap();
    }

    void updateCostmap(const sensor_msgs::PointCloud2& cloud)
    {
        // Clear the previous map.
        for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); ++i)
            {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); ++j)
                    {
                    costmap_->setCost(i, j, costmap_2d::FREE_SPACE);
                    }
            }
        // Ignore points that are close enough to belong to the robot itself.
        const double MIN_OBSTACLE_DISTANCE = 0.2; // Tune according to the robot dimensions; 0.15 m is too small here.
        const double ROBOT_RADIUS = 0.2;  // Robot radius used for inflation.
        // Project each point-cloud point onto the costmap.
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"), iter_y(cloud, "y");
             iter_x != iter_x.end(); ++iter_x, ++iter_y)
        {
            double wx = *iter_x;
            double wy = *iter_y;

            // Filter points belonging to the robot body.
            if (std::hypot(wx, wy) < MIN_OBSTACLE_DISTANCE) {
                continue; // Skip points on the robot itself.
            }
            unsigned int mx, my;

            if (costmap_->worldToMap(wx, wy, mx, my))// worldToMap converts world coordinates to grid indices.
            {// Only laser points inside the costmap can be converted successfully.
                costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);

                // Inflate obstacles by the robot radius, converted to grid cells.
                int radius_cells = static_cast<int>(std::ceil(ROBOT_RADIUS / costmap_->getResolution()));
                inflateCell(mx, my, radius_cells);  // First inflation pass based on the robot radius.
            }
        }

        
        // Apply an additional simple inflation pass.
        const int inflation_radius = 4; // Unit: grid cells.
        for (unsigned int mx = 0; mx < costmap_->getSizeInCellsX(); ++mx)
        {
            for (unsigned int my = 0; my < costmap_->getSizeInCellsY(); ++my)
            {
                if (costmap_->getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE)
                {// getCost reads one byte from the internal costmap array without triggering computation.
                    inflateCell(mx, my, inflation_radius);
                }
            }
        }
        // Compute the EDT after obstacle marking and inflation.
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
        //std::cout << "Grid width: " << grid.info.width << std::endl; // Expected value: 200.
        //std::cout << "Grid origin x: " << grid.info.origin.position.x << std::endl; // Expected value: -5.
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
        // Convert the costmap to an OpenCV binary image: obstacles are 0 and free space is 255.
        cv::Mat binary_map(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_8UC1);
        
        for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); ++y)
        {
            for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); ++x)
            {
                unsigned char cost = costmap_->getCost(x, y);
                // Encode obstacles as 0 and free space as 255.
                binary_map.at<uchar>(y, x) = (cost == costmap_2d::LETHAL_OBSTACLE) ? 0 : 255;
            }
        }

        // Compute the Euclidean distance transform.
        edt_map_.create(binary_map.size(), CV_32FC1);
        cv::distanceTransform(binary_map, edt_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        // Convert distance units from grid cells to metres.
        edt_map_ *= costmap_->getResolution();
        
        std::lock_guard<std::mutex> lock(edt_mutex_);
    }

    // Publish the EDT map for optional visualization.
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
