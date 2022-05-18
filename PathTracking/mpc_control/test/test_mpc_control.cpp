#include <iostream>
#include <vector>
#include <gtest/gtest.h>

#include "cubic_spline_planner/cubic_spline_planner.h"
#include "mpc_control/mpc_control.h"


void getStraightCourse(
    const double& dl,
    std::vector<double>* rx,
    std::vector<double>* ry,
    std::vector<double>* ryaw,
    std::vector<double>* rk) {
  std::vector<double> ax = {0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0};
  std::vector<double> ay = {0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0};
  Spline2D sp(ax, ay);

  sp.calcSplineCourse(dl, rx, ry, ryaw, rk);
}


TEST(test_algorithm, test_mpc_control) {
  double dl = 1.0;
  std::vector<double> rx;
  std::vector<double> ry;
  std::vector<double> ryaw;
  std::vector<double> rk;
  std::vector<double> sp;

  getStraightCourse(dl, &rx, &ry, &ryaw, &rk);

  MPCControl mpc;
  mpc.calcSpeedProfile(rx, ry, ryaw, 0.5, &sp);

  State initial_state(rx[0], ry[0], ryaw[0], 0.0, 0.0);
  mpc.doSimulation(rx, ry, ryaw, rk, sp, dl, initial_state);
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
