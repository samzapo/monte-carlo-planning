#include "Experiment.h"

/*
 *    Multithreading code for running tasks
 *
 *
 */
#include <boost/asio/io_service.hpp>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/optional.hpp>

/// All setable params Here
unsigned NUM_THREADS;
unsigned NUM_SAMPLES;
std::string DURATION_INPUT, STEP_SIZE_INPUT, PARAMETERS;

/// Parse Options
void get_options(int argc, char* argv[]){
  namespace po = boost::program_options;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
  ("help", "produce help message")
  ("threads,j", po::value<unsigned>(&NUM_THREADS)->default_value(1), "set number of worker threads")
  ("samples", po::value<unsigned>(&NUM_SAMPLES)->default_value(0), "set number of samples")
  ("duration", po::value<std::string>(&DURATION_INPUT)->default_value("0.3"), "set duration (virtual time) of each sample")
  ("stepsize,s", po::value<std::string>(&STEP_SIZE_INPUT)->default_value("0.001"), "set step size (virtual time) of each iteration of the simulatior")
  ("parameters", po::value<std::string>(&PARAMETERS)->default_value(""), "set parameters for experiment")
  ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  // Print Help info -- all options descriptions
  if (vm.count("help")) {
    std::cout << desc << "\n";
    exit(1);
  }
}

int main(int argc, char* argv[])
{
  /*
   *  Option Parsing
   */
  get_options(argc,argv);

  /*
   * Create an asio::io_service and a thread_group (thread pool)
   */
//  boost::asio::io_service ioService;
  
  /*
   * This will start the ioService processing loop. All tasks
   * assigned with ioService.post() will start executing.
   */
//  boost::asio::io_service::work work(ioService);
  //boost::optional<boost::asio::io_service::work> work =
  //  boost::in_place(boost::ref(ioService));
  
  /*
   * This will add 2 threads to the thread pool. (You could just put it in a for loop)
   */
//  boost::thread_group threadpool;
//  for(unsigned i = 0;i<NUM_THREADS;i++){
//    threadpool.create_thread(
//                             boost::bind(&boost::asio::io_service::run, &ioService)
//                             );
//  }

  Experiment experiment(NUM_SAMPLES,PARAMETERS);

  /*
   * Assign Tasks
   * This will assign tasks to the thread pool.
   * More about boost::bind: "http://www.boost.org/doc/libs/1_54_0/libs/bind/bind.html#with_functions"
   */
  for (unsigned i=0; i<NUM_SAMPLES; i++)
//    ioService.post(boost::bind(&Experiment::sample,experiment,i));
    experiment.sample(i);
  
  // Let the worker threads notice the posted jobs
  sleep(1);
  
  // this will spin until all posted tasks have completed


  /*
   * This will stop the ioService processing loop. Any tasks
   * you add behind this point will not execute.
   */
//  ioService.stop();
  //work = boost::none;

  /*
   * Will wait till all the treads in the thread pool are finished with
   * their assigned tasks and 'join' them. Just assume the threads inside
   * the threadpool will be destroyed by this method.
   */
//  threadpool.join_all();
  
  experiment.export_data();
  
  return 0;
}
