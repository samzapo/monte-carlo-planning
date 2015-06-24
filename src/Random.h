//
//  Random.h
//  
//
//  Created by samzapo on 6/24/15.
//
//

#ifndef _Random_h
#define _Random_h

/*
#include <random>

class Generator {
  std::default_random_engine generator;
  std::normal_distribution<double> distribution;
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
  
  double generate() {
    double number;
    do{
      number = this->distribution(generator);
    } while (number < this->min && number > this->max);
    return number;
  }
};
*/

#include <gsl_rng.h>

gsl_rng * r =  gsl_rng_alloc( gsl_rng_default );
if( r == NULL ) std::cout << "failed to initialize rng\n";
gsl_rng_set( r, seed );

// generate command values from gaussian
const double SIGMA = GAUSSIAN_STDDEV;
const double MU = GAUSSIAN_MEAN;
for( unsigned i = 0; i < 3; i++ ) {
  walk_force[i] = ;
  walk_torque[i] = 0.0;
}

// free the random number generator



class Generator {
  gsl_rng * generator;
  std::normal_distribution<double> distribution;
  double min;
  double max;
  double mu;
  double sigma;
public:
  Generator(double mean, double stddev, double xmin, double xmax):
  min(xmin),max(xmax),mu(mean),sigma(stddev)
  {
    r =  gsl_rng_alloc( gsl_rng_default );
    if( r == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( r, seed );
  }
  
  // Min and max are 3 std above and below the mean
  Generator(double min, double max):
  min(xmin),max(xmax),mu((min + max) / 2),sigma((max - min) / 6)
  {
    r =  gsl_rng_alloc( gsl_rng_default );
    if( r == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( r, seed );
  }
  
  ~Generator(){
    gsl_rng_free( generator );
  }
  
  double generate() {
    double number;
    do{
      number = gsl_ran_gaussian( generator, sigma ) + mu;
    } while (number < this->min && number > this->max);
    return number;
  }
};

#endif
