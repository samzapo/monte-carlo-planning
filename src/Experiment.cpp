#include "Experiment.h"

#include <stdio.h>
#include <string.h>
#include <string>

std::vector<std::string> Experiment::sample_argv;
std::map<int, unsigned> Experiment::sample_processes;

//typedef std::map<std::string,std::vector<std::vector<double> > > DataMap;
typedef std::map<std::string,boost::shared_ptr<Generator> > ParamMap;
ParamMap Experiment::parameter_generator;

char* const* param_array( std::vector< std::string >& params ) {
  
  const char** pa = (const char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (const char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char* const*) pa;
}

//#include <string>
//#include <stdio.h>
//
//void Experiment::export_data(){
//  FILE * pFile;
//  // Index data
//  for(DataMap::iterator it = data.begin();
//      it != data.end();it++){
//    // Open log file
//    pFile = fopen ((it->first+std::string(".mat")).c_str(),"w");
//    int n = it->second.size();
//    // Index sample
//    for (size_t i = 0; i < n;i++){
//      std::vector<double>& v = it->second[i];
//
//      int m = v.size();
//      // Index element
//      for (int j = 0; j < m;j++){
//        if(j==0)
//          fprintf(pFile, "%f", v[j]);
//        else
//          fprintf(pFile, " %f", v[j]);
//      }
//      fprintf(pFile, "\n" );
//    }
//    fflush(pFile);
//    fclose (pFile);
//  }
//}

#include <boost/algorithm/string.hpp>

void Experiment::init_parameter_generator(std::string& parameters){
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

#include <signal.h>
#include <unistd.h>
#include <map>
#include <time.h>

//-----------------------------------------------------------------------------
// Signal Handling : Simulation process exit detection and
//-----------------------------------------------------------------------------
//void gazebo_c::exit_sighandler( int signum ) { // for action.sa_handler
void Experiment::exit_sighandler( int signum, siginfo_t* info, void* context ) {// for action.sa_sigaction
                                                                                // update exit condition variable
  sample_processes[info->si_pid] = 0;
  //  assert( signum );  // to suppress compiler warning of unused variable
}

#include <sstream>      // std::stringstream
#include <iomanip>      // std::setprecision

//-----------------------------------------------------------------------------
// Multi-core execution : Simulation process spawner
//-----------------------------------------------------------------------------
void Experiment::execute(std::string SAMPLE_BIN, unsigned NUM_SAMPLES, unsigned NUM_THREADS) {
  // install sighandler to detect when gazebo finishes
  // TODO: make sighandler class more compatible with using a member function
  struct sigaction action;
  memset( &action, 0, sizeof(struct sigaction) );
  //  action.sa_handler = exit_sighandler;
  action.sa_sigaction = exit_sighandler; // NEW
  sigaction( SIGCHLD, &action, NULL );
  
  printf( "signal handler installed\n" );
  
  struct timespec nanosleep_req;
  struct timespec nanosleep_rem;
  nanosleep_req.tv_sec = 0;
  nanosleep_req.tv_nsec = 1;
  for (int sample_idx=1;sample_idx<=NUM_SAMPLES; sample_idx++) {
    
    // Generate random set of params for sample in parent process
    std::stringstream parameter_stream;
    parameter_stream << std::setprecision(9);
    bool first = true;
    for(ParamMap::iterator it = parameter_generator.begin();
        it != parameter_generator.end();it++){
      if(first)
        parameter_stream << it->first << "," << it->second->generate();
      else
        parameter_stream << ":" << it->first << "," << it->second->generate();
      first = false;
    }
    std::string parameter_string = parameter_stream.str();
    
    // Run all samples as thir own processes
    pid_t pid = fork();
    if( pid < 0 ) {
      // fork failed
      // TODO: error handling
    }
    
    if( pid == 0 ) {
      // child process
      pid = getpid();

      printf( "Starting Sample (%d) with PID (%d)\n", sample_idx , pid );
      
      sample_argv.push_back("--sample");
      sample_argv.push_back(std::to_string(sample_idx));
      sample_argv.push_back("--parameters");
      // add random set of params for sample to sample_argv (in child)
      sample_argv.push_back(parameter_string);
      
      char* const* exec_argv = param_array(sample_argv);
      
      //    execve( GZSERVER_BIN, exec_argv, exec_envars );
      execv( SAMPLE_BIN.c_str(), exec_argv );
      
      // This code should be unreachable unless exec failed
      perror( "execv" );
      exit( EXIT_FAILURE );
      
    } else {
      // parent process
      
      // Before anything else, register child process in signal map
      sample_processes[pid] = sample_idx;
      nanosleep(&nanosleep_req,&nanosleep_rem);
      
      // yield for a short while to give gazebo time to spin up
      printf( "Experiment yielded to start Sample (%d) with PID (%d)\n", sample_idx , pid );
      
      while( sample_processes.size() >= NUM_THREADS ) {
        usleep(10);
        // Theoretically branch will not be entered; however, detecting EINTR is
        // not fully reliable and this branch is here to ensure exit if the zmq
        // approach fails to cause exit.
        for(std::map<int, unsigned>::iterator it = sample_processes.begin();
            it != sample_processes.end(); it++){
          if( it->second == 0 ) {
            sample_processes.erase(it);
            printf( "Detected Sample (%d) Exit\n", it->first );
            break;
          }
        }
      }
    }
  }
  
  printf( "Experiment Complete\n" );
  
  // uninstall sighandler
  //    action.sa_handler = SIG_DFL;
  action.sa_sigaction = NULL;  // might not be SIG_DFL
  sigaction( SIGCHLD, &action, NULL );
}
