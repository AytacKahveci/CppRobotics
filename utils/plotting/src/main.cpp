// Copyright 2022 Aytac Kahveci
#include <iostream>
#include <unistd.h>
#include "plotting/plotting.h"
#include <boost/thread.hpp>
#include <QApplication>


int main(int argc, char** argv) {
  Plotting p;
  p.init(argc, argv);

  std::vector<double> x, y;
  std::vector<double> cx, cy;
  for (int i = 0; i < 1000; ++i) {
    x.push_back(i + 5);
    y.push_back(i);
    cx.push_back(i);
    cy.push_back(i);
    p.plot(x, y, cx, cy);
    usleep(100000);
  }

  return 0;
}
