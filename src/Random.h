//
//  Random.h
//  
//
//  Created by samzapo on 6/24/15.
//
//

#ifndef _Random_h
#define _Random_h

#include <random>

class Generator {
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution;
  double min;
  double max;
public:
  Generator(double mean, double stddev, double min, double max):
  distribution(mean, stddev), min(min), max(max)
  {}
  
  // Min and max are 3 std above and below the mean
  Generator(double min, double max):
  distribution((min + max) / 2, (max - min) / 6), min(min), max(max)
  {}
  
  double operator ()() {
    double number;
    do{
      number = this->distribution(generator);
    } while (number < this->min && number > this->max)
      return number;
  }
};

#endif
