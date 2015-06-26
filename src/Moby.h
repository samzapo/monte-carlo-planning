//
//  Moby.h
//  
//
//  Created by samzapo on 6/26/15.
//
//

#ifndef _Moby_h
#define _Moby_h

#include <errno.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <Moby/XMLReader.h>
#include <Moby/XMLWriter.h>
#include <Moby/SDFReader.h>

#ifdef USE_OSG
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#endif

#include <Moby/Log.h>
#include <Moby/Simulator.h>
#include <Moby/RigidBody.h>
#include <Moby/EventDrivenSimulator.h>
  

class MobyDriver{
public:
  MobyDriver(){
    /// Beginning iteration for logging
    LOG_START = 0;
    
    /// Ending iteration for logging
    LOG_STOP = std::numeric_limits<unsigned>::max();
    
    /// The logging reporting level
    LOG_REPORTING_LEVEL = 0;
    
    /// The simulation step size
    STEP_SIZE = DEFAULT_STEP_SIZE;
    
    /// The time of the first simulation step
    FIRST_STEP_TIME = -1;
    
    /// The time of the last simulation step
    LAST_STEP_TIME = 0;
    
    /// The current simulation iteration
    ITER = 0;
    
    /// Interval for offscreen renders (0=offscreen renders disabled)
    IMAGE_IVAL = 0;
    
    /// Interval for 3D outputs (0=3D outputs disabled)
    THREED_IVAL = 0;
    
    /// Interval for pickling
    PICKLE_IVAL = 0;
    
    /// Determines whether to do onscreen rendering (false by default)
    ONSCREEN_RENDER = false;
    
    /// Determines whether to output statistics
    OUTPUT_STATS = false;
    
    /// Determines whether to output timings
    OUTPUT_TIMINGS = false;
    
    /// Last pickle iteration
    LAST_PICKLE = -1;
    
    /// Extension/format for 3D outputs (default=Wavefront obj)
//    THREED_EXT = "obj";
    
    /// Determines whether to update graphics (false by default, but certain
    /// options will set to true)
    UPDATE_GRAPHICS = false;
    
    /// The maximum number of iterations (default infinity)
    MAX_ITER = std::numeric_limits<unsigned>::max();
    
    /// The maximum time of the simulation (default infinity)
    MAX_TIME = std::numeric_limits<double>::max();
    
    /// The total (CPU) clock time used by the simulation
    TOTAL_TIME = 0.0;
    
    /// Last 3D output iteration and time output
    LAST_3D_WRITTEN = -1;
    
    /// Last image iteration output
    LAST_IMG_WRITTEN = -1;
    
    /// Outputs to stdout
    OUTPUT_FRAME_RATE = false;
    OUTPUT_ITER_NUM = false;
    OUTPUT_SIM_RATE = false;
    
    /// Render Contact Points
    RENDER_CONTACT_POINTS = false;
  }
  
private:
  /// Handles for dynamic library loading
  std::vector<void*> handles;
  
  /// Horizontal and vertical resolutions for offscreen-rendering
  static const unsigned HORZ_RES = 1024;
  static const unsigned VERT_RES = 768;
  
  /// Beginning iteration for logging
  unsigned LOG_START;
  
  /// Ending iteration for logging
  unsigned LOG_STOP;
  
  /// The logging reporting level
  unsigned LOG_REPORTING_LEVEL;
  
  /// The default simulation step size
  static const double DEFAULT_STEP_SIZE = 0.001;
  
  /// The simulation step size
  double STEP_SIZE;
  
  /// The time of the first simulation step
  double FIRST_STEP_TIME;
  
  /// The time of the last simulation step
  double LAST_STEP_TIME;
  
  /// The current simulation iteration
  unsigned ITER;
  
  /// Interval for offscreen renders (0=offscreen renders disabled)
  unsigned IMAGE_IVAL;
  
  /// Interval for 3D outputs (0=3D outputs disabled)
  int THREED_IVAL;
  
  /// Interval for pickling
  int PICKLE_IVAL;
  
  /// Determines whether to do onscreen rendering (false by default)
  bool ONSCREEN_RENDER;
  
  /// Determines whether to output statistics
  bool OUTPUT_STATS;
  
  /// Determines whether to output timings
  bool OUTPUT_TIMINGS;
  
  /// Last pickle iteration
  unsigned LAST_PICKLE;
  
  /// Extension/format for 3D outputs (default=Wavefront obj)
  char THREED_EXT[5];
  
  /// Determines whether to update graphics (false by default, but certain
  /// options will set to true)
  bool UPDATE_GRAPHICS;
  
  /// The maximum number of iterations (default infinity)
  unsigned MAX_ITER;
  
  /// The maximum time of the simulation (default infinity)
  double MAX_TIME;
  
  /// The total (CPU) clock time used by the simulation
  double TOTAL_TIME;
  
  /// Last 3D output iteration and time output
  unsigned LAST_3D_WRITTEN;
  
  /// Last image iteration output
  unsigned LAST_IMG_WRITTEN;
  
  /// Outputs to stdout
  bool OUTPUT_FRAME_RATE;
  bool OUTPUT_ITER_NUM;
  bool OUTPUT_SIM_RATE;
  
  /// Render Contact Points
  bool RENDER_CONTACT_POINTS;
  
  /// The map of objects read from the simulation XML file
  std::map<std::string, Moby::BasePtr> READ_MAP;
  
#ifdef USE_OSG
  /// The OpenInventor group node for Moby
  osg::Group* MOBY_GROUP;
  
  /// The OpenInventor root group node for this application
  osg::Group* MAIN_GROUP;
  
  /// Pointer to the viewer
  osgViewer::Viewer* viewer_pointer;
#endif
  
  /// Pointer to the controller's initializer, called once (if any)
  typedef void (*init_t)(void*, const std::map<std::string, Moby::BasePtr>&, double);
  std::list<init_t> INIT;
  
  /// Checks whether was compiled with OpenSceneGraph support
  bool check_osg()
  {
#ifdef USE_OSG
    return true;
#else
    return false;
#endif
  }
  
  /*
   * STEPPING
   */
  
  
  /*
   *  INITING MOBY
   */
  
  void read_plugin(const char* filename);
  
  /// Adds lights to the scene when no scene background file specified
  void add_lights()
  {
#ifdef USE_OSG
    // add lights
#endif
  }
  
  /// Gets the XML sub-tree rooted at the specified tag
  boost::shared_ptr<const Moby::XMLTree> find_subtree(boost::shared_ptr<const Moby::XMLTree> root, const std::string& name);
  
  // finds and processes given XML tags
  void process_tag(const std::string& tag, boost::shared_ptr<const Moby::XMLTree> root, void (*fn)(boost::shared_ptr<const Moby::XMLTree>));
  
  /// processes all 'driver' options in the XML file
  void process_xml_options(const std::string& xml_fname);
  
public:
  /// runs the simulator and updates all transforms
  bool step(boost::shared_ptr<Moby::Simulator>& s, unsigned index);
  
  // where everything begins...
  bool init(int argc, char** argv,boost::shared_ptr<Moby::Simulator>& s);
  
  ~MobyDriver(){
    // close the loaded library
    for(size_t i = 0; i < handles.size(); ++i){
      dlclose(handles[i]);
    }
  }
};


#endif
