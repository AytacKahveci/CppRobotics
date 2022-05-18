#include <iostream>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <pure_pursuit/pure_pursuit.h>
#include <cubic_spline_planner/cubic_spline_planner.h>
#include <plotting/plotting.h>

int m_argc;
char** m_argv;

TEST(test_algorithm, test_pure_pursuit) {
  Plotting p;
  p.init(m_argc, m_argv);

  std::vector<double> cx;
  std::vector<double> cy;
  for (double i = 0.0; i <= 50.0; i += 0.5) {
    cx.push_back(i);
    cy.push_back(sin(i / 5.0) * i / 2.0);
  }

  double target_speed = 10.0 / 3.6;
  State state(2.9, 0.0, -3.0, 0.0, 0.0);

  Spline2D sp(cx, cy);
  std::vector<double> rx;
  std::vector<double> ry;
  std::vector<double> ryaw;
  std::vector<double> rk;
  double dl = 0.05;
  sp.calcSplineCourse(dl, &rx, &ry, &ryaw, &rk);


  PurePursuit pp;
  pp.doSimulation(cx, cy, ryaw, rk, dl, target_speed, state, &p);
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  m_argc = argc;
  m_argv = argv;

  return RUN_ALL_TESTS();
}
