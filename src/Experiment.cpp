
#include "Experiment.h"






/*
 *  Init and Data collection
 */

DataMap
  std::map<
          std::string,
          std::vector<Ravelin::VectorNd>
          >

void Experiment::sample(unsigned index){
  std::map< std::string,Ravelin::VectorNd> local_data;
  
  // run sample
  
  
  
  // collect sample data;
  local_data["q"] = q;
  local_data["qd"] = qd;
  
  // Set data into data map:
  for(DataMap::iterator it = data.begin();
      it<data.end();it++){
    it->second[index] = local_data[it->first];
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
    std:string param_name = options[0];
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
  data["q"] = VectorNd(num_samples);
  data["qd"] = VectorNd(num_samples);
}