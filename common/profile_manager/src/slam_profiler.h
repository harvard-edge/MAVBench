#ifndef SLAM_PROFILER_H
#define SLAM_PROFILER_H

#include <vector>
#include <Eigen/Dense>

extern std::vector<Eigen::Matrix4d> P;
extern std::vector<Eigen::Matrix4d> Q;

void collectSLAMData(const std::string& localization_method);

// Functions to calculate SLAM error
// Explanation of formula: "A Benchmark for the Evaluation of RGB-D SLAM Systems"
void transformToSE3Matrix(const tf::StampedTransform& transform, Eigen::Matrix4d& mat);
double relativePoseError(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q);
Eigen::Matrix4d Ei(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q, int i, int delta); 
double rmse_E_delta(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q, int delta);
double absoluteTrajectoryError(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q);

#endif

