//
//  Experiment.h
//  
//
//  Created by samzapo on 6/24/15.
//
//

#ifndef _Experiment_h
#define _Experiment_h

#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "Random.h"

class Experiment{
public:
  
  // FNS
  static
  void execute(std::string SAMPLE_BIN = "sample", unsigned NUM_SAMPLES = 1, unsigned NUM_THREADS = 1);
  
  static
  void init_parameter_generator(std::string& parameters);
  
  // VARS
  static
  std::vector<std::string> sample_argv;
  
private:
  
  // VARS
  static
  std::map<int, unsigned> sample_processes;  // exit condition map for signal handling
  
  static
  std::map< std::string, boost::shared_ptr<Generator> > parameter_generator;
  
  // FNS
  static
  void exit_sighandler(int signum, siginfo_t* info, void* context );
  
};


#endif
