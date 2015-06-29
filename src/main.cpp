#include "Experiment.h"

/*
 *    Multi-process code for running tasks
 *
 *
 */
#include <boost/program_options.hpp>

/// All setable params Here
int main(int argc, char* argv[])
{
  printf("Program options:\n");
  for (int i=0; i<argc; i++) {
    printf("%s ",argv[i]);
  }
  printf("\n");
  /*
   *  Option Parsing
   */
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
  ("help", "produce help message")
  // EXPERIMENT
  ("executable", po::value<std::string>()->default_value("sample"), "EXPERIMENT: set sample executable")
  ("processes,j", po::value<unsigned>()->default_value(1), "EXPERIMENT: set number of simultaneous processes")
  ("samples", po::value<unsigned>()->default_value(1), "EXPERIMENT: set number of samples")
  ("parameters", po::value<std::string>()->default_value(""), "EXPERIMENT: set gaussian normal distributions for sample parameters \"name,min,max:...\"  OR \"name,min,max,mean,stddev:...\"")
  // SAMPLE
  ("duration", po::value<std::string>()->default_value("0.3"), "SAMPLE: set duration (virtual time) of each sample")
  ("stepsize,s", po::value<std::string>()->default_value("0.001"), "SAMPLE: set step size (virtual time) of each iteration of the simulatior")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  // Print Help info -- all options descriptions
  if (vm.count("help")) {
    std::cout << desc << "\n";
    exit(1);
  }
  
  std::string
    param_distribution = vm["parameters"].as<std::string>(),
    sample_exe = vm["executable"].as<std::string>();
  
  
  unsigned
    samples = vm["samples"].as<unsigned>(),
    processes = vm["processes"].as<unsigned>();
  
  /*
   *  Run Experiment
   */
  // Pass arguments to experiment
  
  Experiment::sample_argv.push_back("sample-exec");
  Experiment::sample_argv.push_back("--duration");
  Experiment::sample_argv.push_back(vm["duration"].as<std::string>());
  Experiment::sample_argv.push_back("--stepsize");
  Experiment::sample_argv.push_back(vm["stepsize"].as<std::string>());

  // Set up parameter distributions
  Experiment::init_parameter_generator(param_distribution);
  
  // Execute all samples
  Experiment::execute(sample_exe.c_str(),samples,processes);
  
  // Export final data from experiments
//  Experiment::export_data("experiment.log");

  return 0;
}
