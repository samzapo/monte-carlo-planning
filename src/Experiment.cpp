#include "Experiment.h"

/*
 *  Init and Data collection
 */

typedef std::map<std::string,std::vector<Ravelin::VectorNd> > DataMap;
typedef std::map<std::string,boost::shared_ptr<Generator> > ParamMap;

void Experiment::sample(unsigned index){
  std::map< std::string,Ravelin::VectorNd> local_data;
  
  // Set Parameters
  std::map<std::string,double> params;
  for(ParamMap::iterator it = parameter_generator.begin();
      it != parameter_generator.end();it++){
    params[it->first] = it->second->generate();
  }
  
  // run sample
  
  
  
  // collect sample data;
//  local_data["q"] = q;
//  local_data["qd"] = qd;
  
  // Set data into data map:
  for(DataMap::iterator it = data.begin();
      it != data.end();it++){
    it->second[index] = local_data[it->first];
  }
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