#include <Moby/ControlledBody.h>
#include <Moby/TimeSteppingSimulator.h>
#include <Ravelin/RCArticulatedBodyd.h>
#include <Ravelin/Jointd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>

#include <stdio.h>
#include <set>
#include <string.h>
#include "common.h"
#include "random.h"

#include <boost/shared_ptr.hpp>

using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;
using boost::shared_ptr;
unsigned SAMPLE_NUMBER = 0;
int pid = 0;


namespace Moby {
  extern void close();
  extern bool init(int argc, char** argv, void* s);
  extern bool step(void* s);
}

void apply_transform(Ravelin::Transform3d& T,const std::set<shared_ptr<Jointd> >& outer_joints){
  BOOST_FOREACH(shared_ptr<Jointd> jp, outer_joints){
    shared_ptr<RigidBodyd> rb = jp->get_outboard_link();
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
namespace po = boost::program_options;

void apply_state_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  po::options_description desc("Monte Carlo Method state (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message")
  ("x"    ,   po::value<double>()->default_value(0),  "Absolute linear X Position error [m] applied to Robot")
  ("y"    ,   po::value<double>()->default_value(0),  "Absolute linear Y Position error [m] applied to Robot")
  ("z"    ,   po::value<double>()->default_value(0),  "Absolute linear Z Position error [m] applied to Robot")
  ("roll" ,   po::value<double>()->default_value(0),  "Absolute angular ROLL Position error [rad] applied to Robot")
  ("pitch",   po::value<double>()->default_value(0),  "Absolute angular PITCH Position error [rad] applied to Robot")
  ("yaw"  ,   po::value<double>()->default_value(0),  "Absolute angular YAW Position error [rad] applied to Robot")
  ("dx"   ,   po::value<double>()->default_value(0),  "Absolute linear X Velocity error [m/s] applied to Robot")
  ("dy"   ,   po::value<double>()->default_value(0),  "Absolute linear Y Velocity error [m/s] applied to Robot")
  ("dz"   ,   po::value<double>()->default_value(0),  "Absolute linear Z Velocity error [m/s] applied to Robot")
  ("dax"  ,   po::value<double>()->default_value(0),  "Absolute angular X-AXIS Velocity error [rad/s] applied to Robot")
  ("day"  ,   po::value<double>()->default_value(0),  "Absolute angular Y-AXIS Velocity error [rad/s] applied to Robot")
  ("daz"  ,   po::value<double>()->default_value(0),  "Absolute angular Z-AXIS Velocity error [rad/s] applied to Robot");
  
  
  po::variables_map vm;
  po::parsed_options parsed =
  po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    std::cout << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  
  // Get base Pose
  Pose3d P_base(*(robot->get_base_link()->get_pose().get()));
  // update to relative to global frame
  P_base.update_relative_pose(Moby::GLOBAL);
  // update linear
  P_base.x[0] += vm["x"].as<double>();
  P_base.x[1] += vm["y"].as<double>();
  P_base.x[2] += vm["z"].as<double>();
  // get angular
  double roll,pitch,yaw;
  P_base.q.to_rpy(roll,pitch,yaw);
  // update angular
  roll  += vm["roll"].as<double>();
  pitch += vm["pitch"].as<double>();
  yaw   += vm["yaw"].as<double>();
  // set angular
  P_base.q = Ravelin::Quatd::rpy(roll,pitch,yaw);
  // apply changes
  robot->get_base_link()->set_pose(P_base);
  // Update robot state
  robot->update_link_poses();
  
  // Get robot velocity
  Ravelin::VectorNd qd;
  robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
  int N_JOINT_DOFS = qd.rows()-6;
  // update base velocity
  qd[N_JOINT_DOFS+0] += vm["dx"].as<double>();
  qd[N_JOINT_DOFS+1] += vm["dy"].as<double>();
  qd[N_JOINT_DOFS+2] += vm["dz"].as<double>();
  qd[N_JOINT_DOFS+3] += vm["dax"].as<double>();
  qd[N_JOINT_DOFS+4] += vm["day"].as<double>();
  qd[N_JOINT_DOFS+5] += vm["daz"].as<double>();
  // apply changes
  robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
  // Update robot state
  robot->update_link_velocities();
}

void apply_manufacturing_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  // Add permissible robot parameters
  po::options_description desc("Monte Carlo Method manufacturing (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message");
  
  BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
    std::string name,help;
    name = std::string(rb->body_id+".mass");
    help = std::string("Percent MASS error [%] applied to link: "+rb->body_id);

    desc.add_options()
    (name.c_str(), po::value<double>()->default_value(0),help.c_str());
    
    name = std::string(rb->body_id+".length");
    help = std::string("Percent LENGTH error [%] applied to link: "+rb->body_id);
    
    desc.add_options()
    (name.c_str(), po::value<double>()->default_value(0),help.c_str());
  }
  
  po::variables_map vm;
  po::parsed_options parsed =
  po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    std::cout << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  
  // Perturb mass of robot link by scaling by parameter
  BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
    // Link Mass Adjustment
    if( vm.count(rb->body_id+"mass") ){
      double scale_mass = 1.0 + vm[rb->body_id+"mass"].as<double>();
#ifndef NDEBUG
      printf("Sample %u : Link %s mass *= %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),scale_mass);
#endif
      Ravelin::SpatialRBInertiad Jm(rb->get_inertia());
      Jm.m *= scale_mass;
      rb->set_inertia(Jm);
    }
    
    // Link Length Adjustment
    if( vm.count(rb->body_id+"length") ){
      double scale_length = 1.0 + vm[rb->body_id+"length"].as<double>();
#ifndef NDEBUG
      printf("Sample %u : Link %s length += %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),scale_length);
#endif
      
      shared_ptr<Jointd> jp = rb->get_inner_joint_explicit();
      Pose3d rb_original_pose(*(rb->get_pose().get()));
      rb_original_pose.update_relative_pose(rb->get_pose()->rpose);
      
#ifndef NDEBUG
      std::cout << "Before: " << rb_original_pose << std::endl;
#endif
      // Update transform in link forward direction
      rb_original_pose.x *= scale_length;
#ifndef NDEBUG
      std::cout << "After: " <<  rb_original_pose << std::endl;
#endif
      rb->set_pose(rb_original_pose);
      
      // Update transforms
      jp->Ravelin::Jointd::set_location(jp->get_location(),jp->get_inboard_link(),jp->get_outboard_link());
      // Apply new transform to all children (recursive)
      // NOTE: Doesnt seem to be necessary
      //          apply_transform(T,rb->get_outer_joints());
      
      robot->update_link_poses();
    }
  }
}

void apply_simulator_options(int argc, char* argv[], shared_ptr<Simulator>& sim){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  // INPUT BY EXPERIMENT
  ("sample", po::value<unsigned>(),"Sample Number")
  // INPUT BY USER
  ("duration", po::value<std::string>()->default_value("1"), "set duration (virtual time) of each sample")
  ("stepsize,s", po::value<std::string>()->default_value("0.001"), "set step size (virtual time) of each iteration of the simulatior")
//  ("options", po::value<std::string>()->default_value(""), "other, space-delimited options")
  ;
  
  po::variables_map vm;
//  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed =
    po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);

  po::notify(vm);
  
  // Get sample number for output
  if (vm.count("sample")) {
    SAMPLE_NUMBER = vm["sample"].as<unsigned>();
  } else {
    printf("Sample with PID: %d did not get a sample number", pid);
    exit(1);
  }
  printf("Sample with PID: %d has sample number %u", pid,SAMPLE_NUMBER);

  
  if ( vm.count("help")  )
  {
    std::cout << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  std::string pacer_interface_path(getenv ("PACER_INTERFACE_PATH"));
  
  std::string
  step_size = vm["stepsize"].as<std::string>(),
  duration = vm["duration"].as<std::string>();
  
  //  exit(0);
  /*
   *  Moby Initialization
   */
  
  // run sample
  std::vector<std::string> argvs;
  argvs.push_back("moby-driver");
  // Max time is 0.3 seconds
  argvs.push_back("-mt="+duration);
  argvs.push_back("-s="+step_size);
  // OSG output last frame
  argvs.push_back("-y=osg");
//  argvs.push_back("-v=0");
  // XML output last frame
  //  argvs.push_back("-w=0");
    argvs.push_back("-r");
  argvs.push_back("-p="+pacer_interface_path+"/libPacerMobyPlugin.so");
  argvs.push_back("model.xml");
  //  argvs.push_back("start.xml");
  
  char** moby_argv = param_array_noconst(argvs);
  
  // Ask Moby to init the simulator with options
  Moby::init(argvs.size(), moby_argv,(void*) &sim);
  
  // clean up argv
//  for ( size_t i = 0 ; i < argvs.size() ; i++ )
//    delete [] moby_argv[i];
}

void apply_control_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  // TODO: add randomized force perturbations in simulation
  /*
   typedef std::map<std::string,boost::shared_ptr<Generator> > ParamMap;
   ParamMap parameter_generator;
   
   if(parameter_generator.empty()){
   
   // Create Generators for randomization
   parameter_generator[param_name]
   = boost::shared_ptr<Generator>(new Generator(xmin,xmax));
   
   }
   BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
   // Link Mass Adjustment
   Ravelin::Vector3d perturbation(vm[rb->body_id+".force.x"].as<double>(),
   vm[rb->body_id+".force.y"].as<double>(),
   vm[rb->body_id+".force.z"].as<double>());
   }
   BOOST_FOREACH(shared_ptr<Jointd> j, robot->get_joints()){
   Ravelin::VectorNd perturbation(j->num_dofs());
   perturbation[0] = vm[j->joint_id+".u"].as<double>();
   
   robot->add_force();
   }
   */
}

int main(int argc, char* argv[]){
  // Cet this process's PID for debugging
  pid = getpid();
  
  shared_ptr<Simulator> sim;
  apply_simulator_options(argc,argv,sim);
  /*
   *  Option Parsing
   */
  
  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
  // get event driven simulation and dynamics bodies
  shared_ptr<RCArticulatedBodyd> robot;
  shared_ptr<RigidBodyd> environment;
  
  BOOST_FOREACH(shared_ptr<Moby::ControlledBody> db, sim->get_dynamic_bodies()){
    if(!robot)
      robot = boost::dynamic_pointer_cast<RCArticulatedBodyd>(db);
    if(!environment)
      environment = boost::dynamic_pointer_cast<RigidBodyd>(db);
  }
  
  // Fail if moby was inited wrong
  if(!robot)
    throw std::runtime_error("Could not find robot");
  if(!environment)
    throw std::runtime_error("Could not find environment");
  
  // Apply uncertainty to robot model
  apply_manufacturing_uncertainty(argc, argv,robot);
  
  // Apply uncertainty to robot initial conditions
  apply_state_uncertainty(argc, argv,robot);
  
  /*
   *  Collecting Initial data
   */
  {
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates_euler(q);
    std::cout << "q1 = " << q << std::endl;
    
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
    std::cout << "qd1 = " << qd << std::endl;
  }
  
  /*
   *  Running experiment
   */
  bool stop_sim = false;
  while (!stop_sim) {
    // NOTE: Applied in Pacer -- for now
    // apply_control_uncertainty(argc, argv,robot);
    stop_sim = !Moby::step((void*) &sim);
  }
  
  /*
   *  Collecting final data
   */
  {
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates_euler(q);
    std::cout << "q2 = " << q << std::endl;
    
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
    std::cout << "qd2 = " << qd << std::endl;
  }
  
  // Clean up Moby
  Moby::close();
  
  return 0;
}
