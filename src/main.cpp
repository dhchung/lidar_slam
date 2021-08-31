#include <iostream>
#include <Eigen/Core>
#include <ros/ros.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

int main(int argc, char ** argv) {

    Eigen::Matrix3f A;
    A<<1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout<<A<<std::endl;
    return 0;
}