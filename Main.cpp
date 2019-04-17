/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MyWindow.hpp"
#include "file_ops.hpp"  // readInputFileAsMatrix()

//==============================================================================
void SetKrangComParams(const char* com_params_path,
                       dart::dynamics::SkeletonPtr robot) {
  // Load the parameters from the file
  Eigen::MatrixXd beta;
  try {
    beta = readInputFileAsMatrix(std::string(com_params_path));
  } catch (exception& e) {
    std::cout << e.what() << std::endl;
    assert(false && "Problem loading CoM parameters ... ");
  }

  struct LinkInfo {
    char name[32];  // name of the link in the urdf
    int index;      // order in which it is written in the betaVector file
  };

  std::array<LinkInfo, 22> link_info = {
      {{"Base", 0},      {"LWheel", 1},   {"Spine", 3},     {"Bracket", 4},
       {"L1", 5},        {"L2", 6},       {"L3", 7},        {"L4", 8},
       {"L5", 9},        {"L6", 10},      {"lGripper", 11}, {"lgPlate1", 12},
       {"lgPlate2", 13}, {"R1", 14},      {"R2", 15},       {"R3", 16},
       {"R4", 17},       {"R5", 18},      {"R6", 19},       {"rGripper", 20},
       {"rgPlate1", 21}, {"rgPlate2", 22}}};

  // Set the parameters in the robot
  Eigen::Vector3d bodyMCOM;
  int num_body_params = 4;
  double mi;
  for (const LinkInfo& i : link_info) {
    mi = beta(0, i.index * num_body_params);
    bodyMCOM(0) = beta(0, i.index * num_body_params + 1);
    bodyMCOM(1) = beta(0, i.index * num_body_params + 2);
    bodyMCOM(2) = beta(0, i.index * num_body_params + 3);

    robot->getBodyNode(i.name)->setMass(mi);
    robot->getBodyNode(i.name)->setLocalCOM(bodyMCOM / mi);
  }
}

//==============================================================================
struct KrangInitPoseParams {
  // Initial pose parameters
  double q_base_init;
  double q_waist_init;
  double q_torso_init;
  double q_kinect_init;
  Eigen::Matrix<double, 7, 1> q_left_arm_init;
  Eigen::Matrix<double, 7, 1> q_right_arm_init;

  // To have initial pose as a balanced pose or not
  bool init_with_balance_pose;
};

//==============================================================================
//// Read init pose params
void ReadInitPoseParams(const char* config_file, KrangInitPoseParams* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << "Reading init pose parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Initial base pitch angle
    params->q_base_init = cfg->lookupFloat(scope, "q_base_init");
    std::cout << "q_base_init: " << params->q_base_init << std::endl;

    // Initial waist angle
    params->q_waist_init = cfg->lookupFloat(scope, "q_waist_init");
    std::cout << "q_waist_init: " << params->q_waist_init << std::endl;

    // Initial torso angle
    params->q_torso_init = cfg->lookupFloat(scope, "q_torso_init");
    std::cout << "q_torso_init: " << params->q_torso_init << std::endl;

    // Initial kinect angle
    params->q_kinect_init = cfg->lookupFloat(scope, "q_kinect_init");
    std::cout << "q_kinect_init: " << params->q_kinect_init << std::endl;

    // Initial configuration of the left arm
    str = cfg->lookupString(scope, "q_left_arm_init");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->q_left_arm_init(i);
    stream.clear();
    std::cout << "q_left_arm_init: " << params->q_left_arm_init.transpose()
              << std::endl;

    // Initial configuration of the right arm
    str = cfg->lookupString(scope, "q_right_arm_init");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->q_right_arm_init(i);
    stream.clear();
    std::cout << "q_right_arm_init: " << params->q_right_arm_init.transpose()
              << std::endl;

    // To have initial pose as a balanced pose or not
    params->init_with_balance_pose =
        cfg->lookupBoolean(scope, "init_with_balance_pose");
    std::cout << "init_with_balance_pose: "
              << (params->init_with_balance_pose ? "true" : "false")
              << std::endl;
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading init pose config parameters");
  }
  std::cout << std::endl;
}

//==============================================================================
dart::dynamics::SkeletonPtr createKrang() {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang = loader.parseSkeleton(
      "/usr/local/share/krang/urdf/KrangFixedWheels/krang_fixed_wheel.urdf");
  // loader.parseSkeleton("/usr/local/share/krang/urdf/KrangFixedWheels/krang_fixed_wheel_backup.urdf");
  krang->setName("krang");

  // Set CoM parameters
  SetKrangComParams("/usr/local/share/krang/betaConvergence/bestBetaVector.txt",
                    krang);
  for (int i = 0; i < krang->getNumBodyNodes(); i++) {
    dart::dynamics::BodyNodePtr body = krang->getBodyNode(i);
    std::cout << body->getName() << ": " << body->getMass() << " ";
    std::cout << body->getLocalCOM().transpose() << std::endl;
  }

  KrangInitPoseParams params;
  ReadInitPoseParams("/usr/local/share/krang/fixed_wheels/cfg/params.cfg",
                     &params);
  krang->getJoint("JLWheel")->setPosition(0, params.q_base_init);
  krang->getJoint("JWaist")->setPosition(0, params.q_waist_init);
  krang->getJoint("JTorso")->setPosition(0, params.q_torso_init);
  std::vector<std::string> left_arm_joint_names = {"LJ1", "LJ2", "LJ3", "LJ4",
                                                   "LJ5", "LJ6", "LJFT"};
  std::vector<std::string> right_arm_joint_names = {"RJ1", "RJ2", "RJ3", "RJ4",
                                                    "RJ5", "RJ6", "RJFT"};
  for (int i = 0; i < 7; i++) {
    krang->getJoint(left_arm_joint_names[i])
        ->setPosition(0, params.q_left_arm_init(i));
    krang->getJoint(right_arm_joint_names[i])
        ->setPosition(0, params.q_right_arm_init(i));
  }
  if (params.init_with_balance_pose) {
    Eigen::Vector3d COM;
    COM = krang->getCOM();
    double th = atan2(COM(2), COM(1));

    // Adjust q_base_init to bring COM on top of wheels and set the positions
    // again
    params.q_base_init -= th;
    krang->getJoint("JLWheel")->setPosition(0, params.q_base_init);
  }

  return krang;
}

int main(int argc, char* argv[]) {
  // create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);

  // load skeletons
  dart::utils::DartLoader dl;
  // dart::dynamics::SkeletonPtr ground  =
  // dl.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
  dart::dynamics::SkeletonPtr robot = createKrang();

  // world->addSkeleton(ground); //add ground and robot to the world pointer
  world->addSkeleton(robot);

  // create and initialize the world
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  world->setGravity(gravity);
  world->setTimeStep(1.0 / 1000);

  // create a window and link it to the world
  MyWindow window(new Controller(robot, robot->getBodyNode("lGripper"),
                                 robot->getBodyNode("rGripper")));
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(960, 720, "Forward Simulation");
  glutMainLoop();

  return 0;
}
