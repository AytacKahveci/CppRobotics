#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "cubic_spline_planner/cubic_spline_planner.h"


TEST(test_spline, test_spline) {
  std::vector<double> x = {0.0, 1.0, 2.0, 5.0/2.0};
  std::vector<double> y = {0.0, 1.0, 8.0, 9.0};

  std::vector<double> a_res = {0.0, 1.0, 8.0};
  std::vector<double> b_res = {-12.0/11.0, 57.0/11.0, 48.0/11.0};
  std::vector<double> c_res = {0.0, 69.0/11.0, -78.0/11.0};
  std::vector<double> d_res = {23.0/11.0, -49.0/11.0, 52.0/11.0};

  Spline spline(x, y);
  std::vector<double> a, b, c, d;
  spline.getCoeffs(&a, &b, &c, &d);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(a_res[i], a[i], 1e-6);
    EXPECT_NEAR(b_res[i], b[i], 1e-6);
    EXPECT_NEAR(c_res[i], c[i], 1e-6);
    EXPECT_NEAR(d_res[i], d[i], 1e-6);
  }
}

TEST(test_spline, test_spline2d) {
  std::vector<double> x = {0.0, 1.0, 2.0, 5.0/2.0};
  std::vector<double> y = {0.0, 1.0, 8.0, 9.0};
  std::vector<double> l;
  double tot = 0.0;
  l.push_back(tot);
  for (int i = 0; i < x.size() - 1; ++i) {
    tot += hypotf(x[i+1] - x[i], y[i+1] - y[i]);
    l.push_back(tot);
  }

  Spline2D sp(x, y);

  std::vector<double> rx, ry, ryaw, rk;
  for (size_t i = 0; i < l.size(); ++i) {
    double px, py, pyaw, pcurvature;
    sp.calcPos(l[i], &px, &py);
    sp.calcCurvature(l[i], &pcurvature);
    sp.calcYaw(l[i], &pyaw);

    EXPECT_NEAR(x[i], px, 1e-6);
    EXPECT_NEAR(y[i], py, 1e-6);
  }
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
