/*
 *    Multi-process code for running tasks
 *
 *
 */
#include <vector>
#include <stdio.h>
#include <string>

namespace Experiment{
  extern void execute(const char* SAMPLE_BIN, unsigned NUM_SAMPLES, unsigned NUM_THREADS);
  extern std::vector<std::string> SAMPLE_ARGV;
  extern void init_parameter_generator(const char* path);

};

/// All setable params Here
int main(int argc, char* argv[])
{
  printf("Montecarlo test:\n");
  
  ////////////////// INIT Experiment /////////////////////

  // Name the executable to be processed
  const char* executable = "sample.bin";
  
  // Pass arguments to experiment
  Experiment::SAMPLE_ARGV.push_back(std::string(executable));
  Experiment::SAMPLE_ARGV.push_back("--duration");
  Experiment::SAMPLE_ARGV.push_back("0.5");
  Experiment::SAMPLE_ARGV.push_back("--stepsize");
  Experiment::SAMPLE_ARGV.push_back("0.001");

  // Set up parameter distributions from XML file
  Experiment::init_parameter_generator("parameters.xml");

  // Process "S" samples
  // using "P" processes simultaneously
  unsigned S = 8, P = 8;

  ////////////////// RUN Experiment /////////////////////
  
  // Execute file "executable" relative to working directory
  // Processing "S" samples
  // using "P" processes simultaneously
  Experiment::execute(executable,S,P);
  
  return 0;
}
