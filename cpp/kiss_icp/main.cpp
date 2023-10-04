#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <string>
#include <chrono>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>

// KISS-ICP
// #include "kiss_icp/pipeline/KissICP.hpp"
#include "pipeline/KissICP.hpp"

int main() {
    std::string directoryPath = "/path/to/your/pcd_directory"; // Replace with your directory path that contains .pcd file

    boost::filesystem::path dir(directoryPath);
    boost::filesystem::directory_iterator end_itr;

    kiss_icp::pipeline::KissICP odometry_;
    kiss_icp::pipeline::KISSConfig config_;

    std::vector<Eigen::Vector3d> lidar_frame;
    std::vector<double> timestamps;

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    for (boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr) {
        if (boost::filesystem::is_regular_file(itr->status())) {
            std::string currentFile = itr->path().string();
            if (currentFile.find(".pcd") != std::string::npos) { // Check if it's a .pcd file
                std::cout << "Reading file: " << currentFile << std::endl;

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(currentFile, *cloud) == -1) {
                    PCL_ERROR("Couldn't read file \n");
                }
                // Now, `cloud` contains the point cloud data. Process it as needed.
                for (const auto& point : cloud->points) {
                    lidar_frame.push_back(Eigen::Vector3d(point.x, point.y, point.z));
                }

                // Measure the process time of odometry_.RegisterFrame()
                auto start = std::chrono::high_resolution_clock::now();

                odometry_.RegisterFrame(lidar_frame, timestamps);

                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

            }
        }
    }

    return 0;
}
