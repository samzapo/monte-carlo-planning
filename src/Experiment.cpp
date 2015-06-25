#include "Experiment.h"
/*
 * Moby Control Code
 */
#include <Moby/Simulator.h>

extern bool init(int argc, char** argv,boost::shared_ptr<Moby::Simulator>& s);
extern bool step(boost::shared_ptr<Moby::Simulator>& s);
extern void end();

/*
 *  Init and Data collection
 */

typedef std::map<std::string,std::vector<Ravelin::VectorNd> > DataMap;
typedef std::map<std::string,boost::shared_ptr<Generator> > ParamMap;

char *convert(const std::string & s)
{
  char *pc = new char[s.size()+1];
  std::strcpy(pc, s.c_str());
  return pc;
}

void Experiment::sample(unsigned index){
  /*
   *  Moby Initialization
   */
  boost::shared_ptr<Moby::Simulator> sim;
  // run sample
  std::vector<std::string> moby_options;
  moby_options.push_back("moby-driver");
  // Max time is 0.3 seconds
  moby_options.push_back("-mt=0.3");
  // OSG output last frame
  moby_options.push_back("-y=osg");
  moby_options.push_back("-v=-1");
  // XML output last frame
  moby_options.push_back("-w=-1");
  moby_options.push_back("model.xml");
  std::vector<char*>  argv;
  std::transform(argv.begin(), argv.end(), std::back_inserter(argv), convert);
  
  // apply options and INIT moby
  init(argv.size(),&argv[0],sim);
  
  // clean up argv
  for ( size_t i = 0 ; i < argv.size() ; i++ )
    delete [] argv[i];
  
  // get event driven simulation and dynamics bodies
  boost::shared_ptr<Moby::EventDrivenSimulator>
  eds = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>( sim );
  
  Moby::RCArticulatedBodyPtr robot;
  Moby::RigidBodyPtr environment;
  
  BOOST_FOREACH(Moby::DynamicBodyPtr db, eds->get_dynamic_bodies()){
    if(!robot)
      robot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(db);
    if(!environment)
      environment = boost::dynamic_pointer_cast<Moby::RigidBodyPtr>(db);
  }
  // Fail if moby was inited wrong
  if(!robot)
    throw std::runtime_error("Could not find robot");
  if(!environment)
    throw std::runtime_error("Could not find environment");
  
  /*
   *  Data Collection Initialization
   */
  std::map< std::string,Ravelin::VectorNd> local_data;
  
  /*
   *  Parameter Initialization
   */
  std::map<std::string,double> params;
  for(ParamMap::iterator it = parameter_generator.begin();
      it != parameter_generator.end();it++){
    params[it->first] = it->second->generate();
  }
  
  /*
   *  Parameter Application
   */
  std::cout << "Sample : " << index << "," << num_samples << std::endl;
  for(ParamMap::iterator it = parameter_generator.begin();
      it != parameter_generator.end();it++){
    // Reference (key,value) pair
    std::string& key = it->first;
    double& value = it->second;
    
    std::cout << "\t( " << key << " , " << value << " )" << std::endl;
    
//    if(key.compare("") == 0){
//      
//    }
  }
  
  /*
   *  Running experiment
   */
  bool in_progress = true;
  while (in_progress) {
    in_progress = step(sim);
  }
  
  /*
   *  Collecting final data
   */
  
  //  local_data["q"] = q;
//  local_data["qd"] = qd;
  
  // Set data into data map:
  for(DataMap::iterator it = data.begin();
      it != data.end();it++){
    it->second[index] = local_data[it->first];
  }
  
  // Clean up Moby
  end();
}

#include <string>
#include <stdio.h>

void Experiment::export_data(){
  FILE * pFile;
  // Index data
  for(DataMap::iterator it = data.begin();
      it != data.end();it++){
    // Open log file
    pFile = fopen ((it->first+std::string(".mat")).c_str(),"w");
    int n = it->second.size();
    // Index sample
    for (size_t i = 0; i < n;i++){
      Ravelin::VectorNd& v = it->second[i];

      int m = v.rows();
      // Index element
      for (int j = 0; j < m;j++){
        if(j==0)
          fprintf(pFile, "%f", v[j]);
        else
          fprintf(pFile, " %f", v[j]);
      }
      fprintf(pFile, "\n" );
    }
    fflush(pFile);
    fclose (pFile);
  }
}

#include <boost/algorithm/string.hpp>

void Experiment::init(std::string& parameters){
  
  // Initialize Parameter distributions
  std::vector<std::string> params;
  boost::split(params, parameters, boost::is_any_of(";"));
  size_t N = params.size();
  for (size_t i=0;i<N;i++) {
    std::vector<std::string> options;
    boost::split(options, params, boost::is_any_of(","));
    std::string param_name = options[0];
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
  
  /// Data collection should be hard-coded
  data["q"] = std::vector<Ravelin::VectorNd>(num_samples);
  data["qd"] = std::vector<Ravelin::VectorNd>(num_samples);
}