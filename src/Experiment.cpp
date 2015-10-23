#include <stdio.h>
#include <string.h>
#include <string>

#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "random.h"
#include "common.h"

namespace Experiment{
  
  std::vector<std::string> SAMPLE_ARGV;
  
  //typedef std::map<std::string,std::vector<std::vector<double> > > DataMap;
  typedef std::map<std::string,boost::shared_ptr<Generator> > ParamMap;
  ParamMap parameter_generator;
  
  /*
#include <boost/algorithm/string.hpp>
  
  void init_parameter_generator(std::string& parameters){
    // Initialize Parameter distributions
    std::vector<std::string> params;
    std::string delim1 = ":";
    boost::split(params, parameters, boost::is_any_of(delim1));
    size_t N = params.size();
    for (size_t i=0;i<N;i++) {
      std::vector<std::string> options;
      std::string delim2 = ",";
      boost::split(options, params[i], boost::is_any_of(delim2));
      std::string param_name = options[0];
      std::cout << param_name << " : " << options.size()-1 << " options" << std::endl;
      double xmin,xmax,mu,sigma;
      if(options.size() == 5){
        xmin = std::atof(options[1].c_str());
        xmax = std::atof(options[2].c_str());
        mu = std::atof(options[3].c_str());
        sigma = std::atof(options[4].c_str());
        parameter_generator[param_name]
        = boost::shared_ptr<Generator>(new Generator(mu,sigma,xmin,xmax));
      } else if (options.size() == 3) {
        xmin = std::atof(options[1].c_str());
        xmax = std::atof(options[2].c_str());
        parameter_generator[param_name]
        = boost::shared_ptr<Generator>(new Generator(xmin,xmax));
      } else {
        assert(false);
      }
    }
  }
  */
  
  void init_parameter_generator(const char* path){
    // parse XML description of uncertainty values
    /*
    // Initialize Parameter distributions
    
    xmin = std::atof(options[1].c_str());
    xmax = std::atof(options[2].c_str());
    mu = std::atof(options[3].c_str());
    sigma = std::atof(options[4].c_str());
    
    parameter_generator[param_name]
     = boost::shared_ptr<Generator>(new Generator(xmin,xmax));
     = boost::shared_ptr<Generator>(new Generator(xmin,xmax,mu,sigma));
    */
  }
  
#include <signal.h>
#include <unistd.h>
#include <map>
#include <time.h>
//#include <boost/thread/mutex.hpp>
  
  std::map<int, unsigned> sample_processes;
//  boost::mutex sample_processes_mutex;
  //-----------------------------------------------------------------------------
  // Signal Handling : Simulation process exit detection and
  //-----------------------------------------------------------------------------
  //void gazebo_c::exit_sighandler( int signum ) { // for action.sa_handler
  void exit_sighandler( int signum, siginfo_t* info, void* context ) {
    // for action.sa_sigaction
    
    //lock map mutex
//    sample_processes_mutex.lock();
    // update exit condition variable
    sample_processes[info->si_pid] = 0;
//    sample_processes_mutex.unlock();

    //  assert( signum );  // to suppress compiler warning of unused variable
  }
  
  //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  void execute(const char* SAMPLE_BIN, unsigned NUM_SAMPLES = 1, unsigned NUM_THREADS = 1) {
    // install sighandler to detect when gazebo finishes
    // TODO: make sighandler class more compatible with using a member function
    struct sigaction action;
    memset( &action, 0, sizeof(struct sigaction) );
    //  action.sa_handler = exit_sighandler;
    action.sa_sigaction = exit_sighandler; // NEW
    sigaction( SIGCHLD, &action, NULL );
    
    printf( "signal handler installed\n" );
    
    for (long long unsigned int sample_idx=1;sample_idx<=NUM_SAMPLES; sample_idx++) {
      
      // Generate random set of params for sample in parent process

      std::vector<std::string> PARAMETER_ARGV;
      // NOTE: parallelize this loop
      for(ParamMap::iterator it = parameter_generator.begin(); it != parameter_generator.end();
          it++){
        char buffer[10];
        sprintf(buffer, "%f",it->second->generate());

        PARAMETER_ARGV.push_back("--" + it->first);
        PARAMETER_ARGV.push_back(buffer);
      }
      
      // Run all samples as thir own processes
      pid_t pid = fork();
      if( pid < 0 ) {
        // fork failed
        // TODO: error handling
      }
      
      if( pid == 0 ) {
        // child process
        pid = getpid();
        
        printf( "Starting Sample (%u) with PID (%d)\n", sample_idx , pid );
        
        char buffer[10];
        sprintf(buffer, "%u",sample_idx);
        
        SAMPLE_ARGV.push_back("--sample");
        SAMPLE_ARGV.push_back(buffer);
        // Add PARAMETER_ARGV to SAMPLE_ARGV
        SAMPLE_ARGV.insert(SAMPLE_ARGV.end(), PARAMETER_ARGV.begin(), PARAMETER_ARGV.end());
        
        char* const* exec_argv = param_array(SAMPLE_ARGV);
        
        //    execve( GZSERVER_BIN, exec_argv, exec_envars );
        execve( SAMPLE_BIN , exec_argv, NULL );
        
        // NOTE: unreachable code (memory leak)
        // TODO: clean up argv
        for ( size_t i = 0 ; i <= SAMPLE_ARGV.size() ; i++ )
          delete [] exec_argv[i];

        // This code should be unreachable unless exec failed
        perror( "execve" );
        exit( EXIT_FAILURE );
        
      } else {
        // parent process
        
        // Before anything else, register child process in signal map
        sample_processes[pid] = sample_idx;
        
        // yield for a short while to give everything time to spin up
        struct timespec nanosleep_req;
        struct timespec nanosleep_rem;
        nanosleep_req.tv_sec = 0;
        nanosleep_req.tv_nsec = 1;
        nanosleep(&nanosleep_req,&nanosleep_rem);
        
        printf( "Experiment yielded to start Sample (%d) with PID (%d)\n", sample_idx , pid );
        
        while( sample_processes.size() >= NUM_THREADS ) {
          usleep(10);
          // Theoretically branch will not be entered; however, detecting EINTR is
          // not fully reliable and this branch is here to ensure exit if the zmq
          // approach fails to cause exit.
          typedef std::map<int, unsigned> pid_completed_map;
          
          // NOTE: Do not parallelize this
//          sample_processes_mutex.lock();
          for(pid_completed_map::iterator it = sample_processes.begin(); it != sample_processes.end();it++){
            if( it->second == 0 ) {
              printf( "Detected Sample (%d) Exit\n", it->first );
              sample_processes.erase(it);
              break;
            }
          }
//          sample_processes_mutex.unlock();
        }
      }
    }
    
    printf( "Experiment Complete\n" );
    
    // uninstall sighandler
    //    action.sa_handler = SIG_DFL;
    action.sa_sigaction = NULL;  // might not be SIG_DFL
    sigaction( SIGCHLD, &action, NULL );
  }
}
