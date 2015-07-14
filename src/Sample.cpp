#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>

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
    std::cout << "Sample: " << SAMPLE_NUMBER << " -- " << options[0] << " is perturbed by: " << options[1] << std::endl;
    param_map[options[0]] = std::atof(options[1].c_str());
  }
}

void apply_transform(Ravelin::Transform3d& T,const std::set<Moby::JointPtr>& outer_joints){
  BOOST_FOREACH(Moby::JointPtr jp, outer_joints){
    Moby::RigidBodyPtr rb = jp->get_outboard_link();
    rb->set_pose(T.transform(*(rb->get_pose().get())));
//    jp->set_pose(T.transform(*(jp->get_pose().get())));
//    jp->set_location(
//                     Ravelin::Vector3d(0,0,0,jp->get_pose()),
//                     jp->get_inboard_link(),
//                     jp->get_outboard_link());
    apply_transform(T,rb->get_outer_joints());
    
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
  
//  exit(0);
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
  argvs.push_back("-y=osg");
  argvs.push_back("-v=0");
  // XML output last frame
//  argvs.push_back("-w=0");
  argvs.push_back("-p=/Users/samzapo/Projects/Pacer-experimental/debug/example/interfaces/libPacerMobyPlugin.so");
  argvs.push_back("model.xml");
//  argvs.push_back("start.xml");
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
  //boost::shared_ptr<Moby::EventDrivenSimulator>
  //eds = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>( sim );
  
  Moby::RCArticulatedBodyPtr robot;
  Moby::RigidBodyPtr environment;
  
  BOOST_FOREACH(Moby::DynamicBodyPtr db, sim->get_dynamic_bodies()){
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
    } else if(key.find("mass") == 0){
      std::vector<std::string> parts;
      boost::split(parts, key, boost::is_any_of("."));

      // Perturb mass of robot link by scaling by parameter
      BOOST_FOREACH(Moby::RigidBodyPtr rb, robot->get_links()){
        if(parts[1].compare(rb->id) == 0){
//          printf("Sample %d PID: %d: Link %s mass *= %f\n",SAMPLE_NUMBER, pid,rb->id.c_str(),(1.0+value));
          Ravelin::SpatialRBInertiad Jm(rb->get_inertia());
          Jm.m *= (1.0 + value);
          rb->set_inertia(Jm);
          break;
        }
      }
    } else if(key.find("link-length") == 0){
      std::vector<std::string> parts;
      boost::split(parts, key, boost::is_any_of("."));
      
      // Get legs
      const std::set<Moby::JointPtr>& legs = robot->get_base_link()->get_outer_joints();
      // For each leg
      BOOST_FOREACH(Moby::RigidBodyPtr rb, robot->get_links()){
        // If this is the link we have to edit
        if(parts[1].compare(rb->id) == 0){
//          printf("Sample %d PID: %d: Link %s length += %f\n",SAMPLE_NUMBER, pid,rb->id.c_str(),(value));

          //TRANSFORM3 calc_relative_pose(boost::shared_ptr<const POSE3> source, boost::shared_ptr<const POSE3> target)
          Moby::JointPtr jp = rb->get_inner_joint_explicit();
          boost::shared_ptr<Ravelin::Pose3d> rb_original_pose(new Ravelin::Pose3d(rb->get_pose()));
          rb_original_pose->update_relative_pose(rb->get_pose()->rpose);

//          std::cout << "Before: " << *(rb_original_pose.get()) << std::endl;
          // Update transform in link forward direction
          Ravelin::Origin3d x = rb_original_pose->x;
          x.normalize();
          rb_original_pose->x += (x*value);
//          std::cout << "After: " <<  *(rb_original_pose.get()) << std::endl;
          rb->set_pose(*(rb_original_pose.get()));

          // Update transforms
          jp->Ravelin::Jointd::set_location(jp->get_location());
          // Apply new transform to all children (recursive)
//          apply_transform(T,rb->get_outer_joints());
          robot->update_link_poses();
          break;
        }
      }
    }
  }
  
  {
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates(Moby::DynamicBody::eEuler,q);
    std::cout << "q1 = " << q << std::endl;
    
    robot->get_generalized_velocity(Moby::DynamicBody::eSpatial,qd);
    std::cout << "qd1 = " << qd << std::endl;
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
  {
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates(Moby::DynamicBody::eEuler,q);
    std::cout << "q2 = " << q << std::endl;
    
    robot->get_generalized_velocity(Moby::DynamicBody::eSpatial,qd);
    std::cout << "qd2 = " << qd << std::endl;
  }
  
  // Clean up Moby
  end();
  
  return 0;
}
