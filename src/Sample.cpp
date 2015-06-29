#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>

#include <stdio.h>
#include <string.h>

#include <boost/shared_ptr.hpp>


unsigned SAMPLE_NUMBER = 0;

char *convert(const std::string & s)
{
  char *pc = new char[s.size()+1];
  strcpy(pc, s.c_str());
  return pc;
}

extern void end();
extern bool init(int argc, char** argv, boost::shared_ptr<Moby::Simulator>& s);
extern bool step(boost::shared_ptr<Moby::Simulator>& s,unsigned INDEX);

#include <boost/algorithm/string.hpp>

void parse_parameters(std::string param_string, std::map<std::string, double>& param_map){
  // Initialize Parameter distributions
  std::vector<std::string> params;
  const std::string delim1 = ":";
  boost::split(params, param_string, boost::is_any_of(delim1));
  size_t N = params.size();
  for (size_t i=0;i<N;i++) {
    std::vector<std::string> options;
    const std::string delim2 = ",";
    boost::split(options, params[i], boost::is_any_of(delim2));
    printf("Sample: %d -- %s is perturbed by: %s", SAMPLE_NUMBER,options[0].c_str(),options[1].c_str());
    param_map[options[0]] = std::atof(options[1].c_str());
  }
}

#include <unistd.h>
#include <boost/program_options.hpp>

int main(int argc, char* argv[]){
  // Cet this process's PID for debugging
  int pid = getpid();

  /*
   *  Option Parsing
   */
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
  ("help", "produce help message")
  // INPUT BY EXPERIMENT
  ("sample", po::value<unsigned>(),"")
  ("parameters", po::value<std::string>(),"")
  // INPUT BY USER
  ("duration", po::value<std::string>()->default_value("0.3"), "set duration (virtual time) of each sample")
  ("stepsize,s", po::value<std::string>()->default_value("0.001"), "set step size (virtual time) of each iteration of the simulatior")
  ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  // Get sample number for output
  if (vm.count("sample")) {
    SAMPLE_NUMBER = vm["sample"].as<unsigned>();
  } else {
    printf("Sample with PID: %d did not get a sample number", pid);
    exit(1);
  }
  
  // Get sample parameters
  std::map<std::string, double> param_map;
  if (vm.count("parameters")) {
    parse_parameters(vm["parameters"].as<std::string>(),param_map);
  } else {
    printf("Sample %d PID: %d did not get a set of parameters",SAMPLE_NUMBER, pid);
    exit(1);
  }
  
  std::string
    step_size = vm["stepsize"].as<std::string>(),
    duration = vm["duration"].as<std::string>();
  
  /*
   *  Moby Initialization
   */
  boost::shared_ptr<Moby::Simulator> sim;
  
  
  // run sample
  std::vector<std::string> argvs;
  argvs.push_back("moby-driver");
  // Max time is 0.3 seconds
  argvs.push_back("-mt="+duration);
  argvs.push_back("-s="+step_size);
  // OSG output last frame
  //argvs.push_back("-y=osg");
  //argvs.push_back("-v=0");
  // XML output last frame
  //argvs.push_back("-w=0");
  //  argvs.push_back("-p=/Users/samzapo/Projects/Pacer/build/example/interfaces/libPacerMobyPlugin.so");
  argvs.push_back("model.xml");
  //argvs.push_back("start.xml");
  std::vector<char*>  moby_argv;
  std::transform(argvs.begin(), argvs.end(), std::back_inserter(moby_argv), convert);
  
  // apply options and INIT moby
  init(moby_argv.size(),&moby_argv[0],sim);
  
  //TODO: clean up argv
  for ( size_t i = 0 ; i < moby_argv.size() ; i++ )
    delete [] moby_argv[i];
  
  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
  // get event driven simulation and dynamics bodies
  boost::shared_ptr<Moby::EventDrivenSimulator>
  eds = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>( sim );
  
  Moby::RCArticulatedBodyPtr robot;
  Moby::RigidBodyPtr environment;
  
  BOOST_FOREACH(Moby::DynamicBodyPtr db, eds->get_dynamic_bodies()){
    if(!robot)
      robot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(db);
    if(!environment)
      environment = boost::dynamic_pointer_cast<Moby::RigidBody>(db);
  }
  // Fail if moby was inited wrong
  if(!robot)
    throw std::runtime_error("Could not find robot");
  if(!environment)
    throw std::runtime_error("Could not find environment");

  /*
   *  Parameter Application
   */
//  std::cout << "Sample : " << index << "," << num_samples << std::endl;
  for(std::map<std::string,double>::iterator it = param_map.begin();
      it != param_map.end();it++){
    // Reference (key,value) pair
    const std::string& key = it->first;
    const double& value = it->second;
    
    std::cout << "\t( " << key << " , " << value << " )" << std::endl;
    
    if(key.compare("x") == 0){
      Ravelin::VectorNd q;
      robot->get_generalized_coordinates(Moby::DynamicBody::eEuler,q);
      int i = q.rows()- 1 - 4 - 2;
      q[i] = q[i] + value;
      robot->set_generalized_coordinates(Moby::DynamicBody::eEuler,q);
    }
    else if(key.compare("y") == 0){
      Ravelin::VectorNd q;
      robot->get_generalized_coordinates(Moby::DynamicBody::eEuler,q);
      int i = q.rows()- 1 - 4 - 1;
      q[i] = q[i] + value;
      robot->set_generalized_coordinates(Moby::DynamicBody::eEuler,q);
    }
    else if(key.compare("z") == 0){
      Ravelin::VectorNd q;
      robot->get_generalized_coordinates(Moby::DynamicBody::eEuler,q);
      int i = q.rows()- 1 - 4;
      q[i] = q[i] + value;
      robot->set_generalized_coordinates(Moby::DynamicBody::eEuler,q);
    }
  }
  
  /*
   *  Running experiment
   */
  bool in_progress = true;
  while (in_progress) {
    in_progress = step(sim,SAMPLE_NUMBER);
  }
  
  /*
   *  Collecting final data
   */
//  std::map< std::string,std::vector<double> > local_data;
  
  Ravelin::VectorNd q,qd;
  robot->get_generalized_coordinates(Moby::DynamicBody::eEuler,q);
  std::cout << "q = " << q << std::endl;
  
  //local_data["q"] = q;
  robot->get_generalized_velocity(Moby::DynamicBody::eSpatial,qd);
  std::cout << "qd = " << qd << std::endl;
  //local_data["qd"] = qd;
  
  // Clean up Moby
  end();
  
  return 0;
}