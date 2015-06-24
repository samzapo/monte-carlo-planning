//
//  Experiment.h
//  
//
//  Created by samzapo on 6/24/15.
//
//

#ifndef _Experiment_h
#define _Experiment_h

#include "Random.h"
#include <map>
#include <vector>
#include <Ravelin/VectorNd.h>

class Experiment{
public:
  Experiment(unsigned NUM_SAMPLES, std::string& parameters) : num_samples(NUM_SAMPLES){
    init(parameters);
  };
  
  void sample(unsigned index);
  void export_data();
private:
  unsigned num_samples;
  
  std::map< std::string,
            std::vector<Ravelin::VectorNd> > data;
  
  std::map< std::string,
            boost::shared_ptr<Generator> > parameter_generator;
  
  void init(std::string& parameters);
};


#endif
