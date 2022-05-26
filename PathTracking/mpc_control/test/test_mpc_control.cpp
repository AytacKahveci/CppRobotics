#include <iostream>
#include <vector>
#include <unistd.h>
#include <gtest/gtest.h>

#include "cubic_spline_planner/cubic_spline_planner.h"
#include "mpc_control/mpc_control.h"
#include <plotting/plotting.h>

int m_argc;
char** m_argv;

void getStraightCourse(
    const double& dl,
    std::vector<double>* rx,
    std::vector<double>* ry,
    std::vector<double>* ryaw,
    std::vector<double>* rk) {
  std::vector<double> cx;
  std::vector<double> cy;
  for (double i = 0.0; i <= 50.0; i += 0.5) {
    cx.push_back(i);
    cy.push_back(sin(i / 5.0) * i / 2.0);
  }
  Spline2D sp(cx, cy);

  sp.calcSplineCourse(dl, rx, ry, ryaw, rk);
}


TEST(test_algorithm, test_mpc_control) {
  Plotting p;
  p.init(m_argc, m_argv);

  double dl = 1.0;
  std::vector<double> rx;
  std::vector<double> ry;
  std::vector<double> ryaw;
  std::vector<double> rk;
  std::vector<double> sp;

  getStraightCourse(dl, &rx, &ry, &ryaw, &rk);
  /* while (true)
  {
    p.plot(rx, ry, rx, ry);
    usleep(100000);
  } */
  MPCControl mpc;
  mpc.calcSpeedProfile(rx, ry, ryaw, 0.5, &sp);

  State initial_state(rx[0], ry[0], ryaw[0], 0.0, 0.0);
  mpc.doSimulation(rx, ry, ryaw, rk, sp, dl, initial_state, &p);
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  m_argc = argc;
  m_argv = argv;

  return RUN_ALL_TESTS();
}
