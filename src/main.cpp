#include "Experiment.h"

/*
 *    Multithreading code for running tasks
 *
 *
 */
#include <boost/asio/io_service.hpp>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

/// All setable params Here
unsigned NUM_THREADS;
unsigned NUM_SAMPLES;
std::string PARAMETERS;

/// Parse Options
void get_options(int argc, char* argv[]){
  namespace po = boost::program_options;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
  ("help", "produce help message")
  ("threads,j", po::value<unsigned>(&NUM_THREADS)->default_value(1), "set number of worker threads [DEFAULT: 1]")
  ("samples", po::value<unsigned>(&NUM_SAMPLES)->default_value(0), "set number of worker threads [DEFAULT: 0]")
  ("parameters", po::value<std::string>(&PARAMETERS)->default_value(""), "set parameters for experiment [DEFAULT: \"\"]")
  ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  // Print Help info -- all options descriptions
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }
  
  return vm;
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
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  
  /*
   * This will start the ioService processing loop. All tasks
   * assigned with ioService.post() will start executing.
   */
  boost::asio::io_service::work work(ioService);
  
  /*
   * This will add 2 threads to the thread pool. (You could just put it in a for loop)
   */
  for(unsigned i = 0;i<NUM_THREADS;i++){
    threadpool.create_thread(
                             boost::bind(&boost::asio::io_service::run, &ioService)
                             );
  }

  Experiment experiment(NUM_SAMPLES,PARAMETERS);
  /*
   * Assign Tasks
   * This will assign tasks to the thread pool.
   * More about boost::bind: "http://www.boost.org/doc/libs/1_54_0/libs/bind/bind.html#with_functions"
   */
  for (unsigned i=0; i<NUM_SAMPLES; i++)
    ioService.post(boost::bind(experiment.sample,_1)(i));
  
  /*
   * This will stop the ioService processing loop. Any tasks
   * you add behind this point will not execute.
   */
  ioService.stop();

  /*
   * Will wait till all the treads in the thread pool are finished with
   * their assigned tasks and 'join' them. Just assume the threads inside
   * the threadpool will be destroyed by this method.
   */
  threadpool.join_all();
  
  experiment.compile_data();
}