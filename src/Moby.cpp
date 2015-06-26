//
//  Moby.cpp
//  
//
//  Created by samzapo on 6/25/15.
//
//

/*****************************************************************************
 * The "driver" program described in the README file.
 *****************************************************************************/

#include "Moby.h"
#include "Random.h"

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

/*
/// processes the 'camera' tag
void process_camera_tag(shared_ptr<const Moby::XMLTree> node)
{
//  if (!ONSCREEN_RENDER)
//    return;
  
  // read all attributes
  XMLAttrib* target_attr = node->get_attrib("target");
  XMLAttrib* position_attr = node->get_attrib("position");
  XMLAttrib* up_attr = node->get_attrib("up");
  if (!target_attr || !position_attr || !up_attr)
    return;
  
  // get the actual values
  Vector3d up;
  Point3d target, position;
  target_attr->get_vector_value(target);
  position_attr->get_vector_value(position);
  up_attr->get_vector_value(up);
  
  // setup osg vectors
#ifdef USE_OSG
  osg::Vec3d position_osg(position[0], position[1], position[2]);
  osg::Vec3d target_osg(target[0], target[1], target[2]);
  osg::Vec3d up_osg(up[0], up[1], up[2]);
  
  // set the camera view
  if (viewer_pointer && viewer_pointer->getCameraManipulator())
  {
    osg::Camera* camera = viewer_pointer->getCamera();
    camera->setViewMatrixAsLookAt(position_osg, target_osg, up_osg);
    
    // setup the manipulator using the camera, if necessary
    viewer_pointer->getCameraManipulator()->setHomePosition(position_osg, target_osg, up_osg);
  }
#endif
}
/// processes the 'window' tag
void process_window_tag(shared_ptr<const Moby::XMLTree> node)
{
  // don't process if not onscreen rendering
//  if (!ONSCREEN_RENDER)
//    return;
  
#ifdef USE_OSG
  // read window location
  XMLAttrib* loc_attr = node->get_attrib("location");
  
  // read window size
  XMLAttrib* size_attr = node->get_attrib("size");
  
  // get the actual values
  Vector2d loc(0,0), size(640,480);
  if (loc_attr)
    loc_attr->get_vector_value(loc);
  if (size_attr)
    size_attr->get_vector_value(size);
  
  // setup the window
  viewer_pointer->setUpViewInWindow(loc[0], loc[1], size[0], size[1]);
#endif
}
*/

  /*
   *  INITING MOBY
   */

  void MobyDriver::read_plugin(const char* filename)
  {
    // attempt to read the file
    void* plugin = dlopen(filename, RTLD_LAZY);
    if (!plugin)
    {
      // get the error string, in case we need it
      char* dlerror_str = dlerror();
      
      // attempt to use the plugin path
      char* plugin_path = getenv("MOBY_PLUGIN_PATH");
      if (plugin_path)
      {
        // get the plugin path and make sure it has a path string at the end
        std::string plugin_path_str(plugin_path);
        if (plugin_path_str.at(plugin_path_str.size()-1) != '/')
          plugin_path_str += '/';
        
        // concatenate
        plugin_path_str += filename;
        
        // attempt to re-open the plugin
        plugin = dlopen(plugin_path_str.c_str(), RTLD_LAZY);
      }
      
      // check whether the plugin was successfully loaded
      if (!plugin)
      {
        std::cerr << "driver: failed to read plugin from " << filename << std::endl;
        std::cerr << "  " << dlerror_str << std::endl;
        exit(-1);
      }
    }
    handles.push_back(plugin);
    
    // attempt to load the initializer
    dlerror();
    INIT.push_back((init_t) dlsym(plugin, "init"));
    const char* dlsym_error = dlerror();
    if (dlsym_error)
    {
      std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
      exit(-1);
    }
  }

  /// Gets the XML sub-tree rooted at the specified tag
  shared_ptr<const Moby::XMLTree> MobyDriver::find_subtree(shared_ptr<const Moby::XMLTree> root, const std::string& name)
  {
    // if we found the tree, return it
    if (strcasecmp(root->name.c_str(), name.c_str()) == 0)
      return root;
    
    // otherwise, look for it recursively
    const std::list<Moby::XMLTreePtr>& children = root->children;
    for (std::list<Moby::XMLTreePtr>::const_iterator i = children.begin(); i != children.end(); i++)
    {
      shared_ptr<const Moby::XMLTree> node = find_subtree(*i, name);
      if (node)
        return node;
    }
    
    // return NULL if we are here
    return shared_ptr<const Moby::XMLTree>();
  }
  
  // finds and processes given XML tags
  void MobyDriver::process_tag(const std::string& tag, shared_ptr<const Moby::XMLTree> root, void (*fn)(shared_ptr<const Moby::XMLTree>))
  {
    // if this node is of the given type, process it
    if (strcasecmp(root->name.c_str(), tag.c_str()) == 0)
      fn(root);
    else
    {
      const std::list<Moby::XMLTreePtr>& child_nodes = root->children;
      for (std::list<Moby::XMLTreePtr>::const_iterator i = child_nodes.begin(); i != child_nodes.end(); i++)
        process_tag(tag, *i, fn);
    }
  }
  
  /// processes all 'driver' options in the XML file
  void MobyDriver::process_xml_options(const std::string& xml_fname)
  {
    // *************************************************************
    // going to remove any path from the argument and change to that
    // path; this is done so that all files referenced from the
    // local path of the XML file are found
    // *************************************************************
    
    // set the filename to use as the argument, by default
    std::string filename = xml_fname;
    
    // get the current pathname
    size_t BUFSIZE = 128;
    boost::shared_array<char> cwd;
    while (true)
    {
      cwd = boost::shared_array<char>((new char[BUFSIZE]));
      if (getcwd(cwd.get(), BUFSIZE) == cwd.get())
        break;
      if (errno != ERANGE)
      {
        std::cerr << "process_xml_options() - unable to allocate sufficient memory!" << std::endl;
        return;
      }
      BUFSIZE *= 2;
    }
    
    // separate the path from the filename
    size_t last_path_sep = xml_fname.find_last_of('/');
    if (last_path_sep != std::string::npos)
    {
      // get the new working path
      std::string pathname = xml_fname.substr(0,last_path_sep+1);
      
      // change to the new working path
      chdir(pathname.c_str());
      
      // get the new filename
      filename = xml_fname.substr(last_path_sep+1,std::string::npos);
    }
    
    // read the XML Tree
    shared_ptr<const Moby::XMLTree> driver_tree = Moby::XMLTree::read_from_xml(filename);
    if (!driver_tree)
    {
      std::cerr << "process_xml_options() - unable to open file " << xml_fname;
      std::cerr << " for reading" << std::endl;
      chdir(cwd.get());
      return;
    }
    
    // find the driver tree
    driver_tree = find_subtree(driver_tree, "driver");
    
    // make sure that the driver node was found
    if (!driver_tree)
    {
      chdir(cwd.get());
      return;
    }
    
    // process tags
   // if (ONSCREEN_RENDER){
   //   process_tag("window", driver_tree, process_window_tag);
   //   process_tag("camera", driver_tree, process_camera_tag);
   // }
    
    // change back to current directory
    chdir(cwd.get());
  }
  
  /// runs the simulator and updates all transforms
  bool MobyDriver::step(boost::shared_ptr<Simulator>& s, unsigned index)
  {
  #ifdef USE_OSG
  //  if (ONSCREEN_RENDER)
  //  {
  //    if (viewer.done())
  //      break;
  //    viewer.frame();
  //  }
  #endif
    // get the simulator pointer
  //  boost::shared_ptr<Simulator> s = *(boost::shared_ptr<Simulator>*) arg;
    
    // get the simulator as event driven simulation
    boost::shared_ptr<EventDrivenSimulator> eds = boost::dynamic_pointer_cast<EventDrivenSimulator>( s );
    
    // output the iteration #
  #ifndef NDEBUG
    std::cout << "iteration: " << ITER << "  simulation time: " << s->current_time << std::endl;
  #endif
    
  // only update the graphics if it is necessary; update visualization first
  // in case simulator takes some time to perform first step
  if (UPDATE_GRAPHICS){
    std::cout << "Update Graphics" << std::endl;
    s->update_visualization();
  }
    
    // output the image, if desired
  #ifdef USE_OSG
    // output the 3D file, if desired
    if (THREED_IVAL > 0)
    {
      // determine at what iteration nearest frame would be output
      if (ITER % THREED_IVAL == 0)
      {
        // write the file (fails silently)
        char buffer[128];
        sprintf(buffer, "driver.out-%08u-%f-sample%d.%s", ++LAST_3D_WRITTEN, s->current_time, index,THREED_EXT);
        osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
        std::cout << "output 3D at STEP" << std::endl;
      }
    }
  #endif
    
    // serialize the simulation, if desired
    if (PICKLE_IVAL > 0)
    {
      // determine at what iteration nearest pickle would be output
      if (ITER % PICKLE_IVAL == 0)
      {
        // write the file (fails silently)
        char buffer[128];
        sprintf(buffer, "driver.out-%08u-%f-sample%d.xml", ++LAST_PICKLE, s->current_time, index);
        XMLWriter::serialize_to_xml(std::string(buffer), s);
        std::cout << "output XML at STEP" << std::endl;
      }
    }
    
    // step the simulator
    clock_t pre_sim_t = clock();
    s->step(STEP_SIZE);
    clock_t post_sim_t = clock();
    double total_t = (post_sim_t - pre_sim_t) / (double) CLOCKS_PER_SEC;
    TOTAL_TIME += total_t;
    
    // output the iteration / stepping rate
    std::cout << "time to compute last iteration: " << total_t << " (" << TOTAL_TIME / ITER << "s/iter, " << TOTAL_TIME / s->current_time << "s/step)" << std::endl;
    
    // update the iteration #
    ITER++;
    
    // check that maximum number of iterations or maximum time not exceeded
    if (ITER >= MAX_ITER || s->current_time > MAX_TIME){
      if(PICKLE_IVAL == 0){
        // write the file (fails silently)
        char buffer[128];
        sprintf(buffer, "state-%f-sample%d.xml", s->current_time, index);
        XMLWriter::serialize_to_xml(std::string(buffer), s);
        std::cout << "output XML at END" << std::endl;
      }
  #ifdef USE_OSG
      if(THREED_IVAL == 0){
        // write the file (fails silently)
        char buffer[128];
        sprintf(buffer, "state-%f-sample%d.%s", s->current_time, index, THREED_EXT);
        osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
        std::cout << "output 3D at END" << std::endl;
      }
  #endif
      return false;
    }
  #ifdef USE_OSG
    //usleep(DYNAMICS_FREQ);
  #endif
    return true;
  }

  // where everything begins...
  bool MobyDriver::init(int argc, char** argv,boost::shared_ptr<Simulator>& s)
  {
    const unsigned ONECHAR_ARG = 3, TWOCHAR_ARG = 4;
    
  #ifdef USE_OSG
    const double DYNAMICS_FREQ = 0.001;
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer_pointer = &viewer;
  #endif
    
    // setup some default options
    std::string scene_path;
    THREED_IVAL = 0;
    IMAGE_IVAL = 0;
    
    // check that syntax is ok
    if (argc < 2)
    {
      std::cerr << "syntax: driver [OPTIONS] <xml file>" << std::endl;
      std::cerr << "        (see README for OPTIONS)" << std::endl;
      return false;
    }
    
    // get all options
    for (int i=1; i< argc-1; i++)
    {
      // get the option
      std::string option(argv[i]);
      
      // process options
      if (option == "-r")
      {
        ONSCREEN_RENDER = true;
        UPDATE_GRAPHICS = true;
        check_osg();
      }
      else if (option.find("-of") != std::string::npos)
        OUTPUT_FRAME_RATE = true;
      else if (option.find("-ot") != std::string::npos)
      {
        std::cout << "timings/statistics information follows:" << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "column 1:  iteration" << std::endl;
        std::cout << "column 2:  semi-implicit Euler steps / processing time" << std::endl;
        std::cout << "column 3:  general integration steps interrupted by invalid state / processing time" << std::endl;
        std::cout << "column 4:  general integration steps interrupted by invalid velocity / processing time" << std::endl;
        std::cout << "column 5:  failed acceleration solving events / processing time" << std::endl;
        std::cout << "column 6:  limit estimates exceeded / processing time" << std::endl;
        std::cout << "column 7:  successful integration calls / processing time" << std::endl;
        std::cout << "column 8: min/mean/max non-Euler, integration steps" << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
        OUTPUT_TIMINGS = true;
      }
      else if (option.find("-os") != std::string::npos)
        OUTPUT_STATS = true;
      else if (option.find("-oi") != std::string::npos)
        OUTPUT_ITER_NUM = true;
      else if (option.find("-or") != std::string::npos)
        OUTPUT_SIM_RATE = true;
      else if (option.find("-w=") != std::string::npos)
      {
        PICKLE_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
        assert(PICKLE_IVAL >= 0);
      }
      else if (option.find("-v=") != std::string::npos)
      {
        UPDATE_GRAPHICS = true;
        check_osg();
        THREED_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
        assert(THREED_IVAL >= 0);
      }
      else if (option.find("-i=") != std::string::npos)
      {
        check_osg();
        UPDATE_GRAPHICS = true;
        IMAGE_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
        assert(IMAGE_IVAL >= 0);
      }
      else if (option.find("-s=") != std::string::npos)
      {
        STEP_SIZE = std::atof(&argv[i][ONECHAR_ARG]);
      }
      else if (option.find("-lf=") != std::string::npos)
      {
        std::string fname(&argv[i][TWOCHAR_ARG]);
        OutputToFile::stream.open(fname.c_str());
      }
      else if (option.find("-l=") != std::string::npos)
      {
        LOG_REPORTING_LEVEL = std::atoi(&argv[i][ONECHAR_ARG]);
        Log<OutputToFile>::reporting_level = LOG_REPORTING_LEVEL;
      }
      else if (option.find("-lt=") != std::string::npos)
      {
        LOG_START = std::atoi(&argv[i][TWOCHAR_ARG]);
      }
      else if (option.find("-lp=") != std::string::npos)
      {
        LOG_STOP = std::atoi(&argv[i][TWOCHAR_ARG]);
      }
      else if (option.find("-mi=") != std::string::npos)
      {
        MAX_ITER = std::atoi(&argv[i][TWOCHAR_ARG]);
        assert(MAX_ITER > 0);
      }
      else if (option.find("-mt=") != std::string::npos)
      {
        MAX_TIME = std::atof(&argv[i][TWOCHAR_ARG]);
        assert(MAX_TIME > 0);
      }
      else if (option.find("-x=") != std::string::npos)
      {
        check_osg();
        scene_path = std::string(&argv[i][ONECHAR_ARG]);
      }
      else if (option.find("-p=") != std::string::npos)
      {
        std::vector<std::string> plugins;
        std::string arg = std::string(&argv[i][ONECHAR_ARG]);
        boost::split(plugins, arg, boost::is_any_of(","));
        for(size_t i = 0; i < plugins.size(); ++i){
          read_plugin(plugins[i].c_str());
        }
      }
      else if (option.find("-y=") != std::string::npos)
      {
        strcpy(THREED_EXT, &argv[i][ONECHAR_ARG]);
      } else if (option.find("-vcp") != std::string::npos)
        RENDER_CONTACT_POINTS = true;
      
    }
    
    // setup the simulation
    if (std::string(argv[argc-1]).find(".xml") != std::string::npos)
      READ_MAP = XMLReader::read(std::string(argv[argc-1]));
    else if (std::string(argv[argc-1]).find(".sdf") != std::string::npos)
    {
      // artificially create the read map
      shared_ptr<EventDrivenSimulator> eds = SDFReader::read(std::string(argv[argc-1]));
      READ_MAP[eds->id] = eds;
      BOOST_FOREACH(DynamicBodyPtr db, eds->get_dynamic_bodies())
      READ_MAP[db->id] = db;
    }
    
    // setup the offscreen renderer if necessary
  #ifdef USE_OSG
    if (IMAGE_IVAL > 0)
    {
      // TODO: setup offscreen renderer here
    }
  #endif
    
    // get the (only) simulation object
  //  boost::shared_ptr<Simulator> s;
    for (std::map<std::string, Moby::BasePtr>::const_iterator i = READ_MAP.begin(); i != READ_MAP.end(); i++)
    {
      s = boost::dynamic_pointer_cast<Simulator>(i->second);
      if (s)
        break;
    }
    
    // make sure that a simulator was found
    if (!s)
    {
      std::cerr << "driver: no simulator found in " << argv[argc-1] << std::endl;
      return false;
    }
    
    // setup osg window if desired
  #ifdef USE_OSG
    if (ONSCREEN_RENDER)
    {
      // setup any necessary handlers here
      viewer.setCameraManipulator(new osgGA::TrackballManipulator());
      viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
      viewer.addEventHandler(new osgViewer::WindowSizeHandler);
      viewer.addEventHandler(new osgViewer::StatsHandler);
    }
    
    // init the main group
    MAIN_GROUP = new osg::Group;
  #endif
    
    // call the initializers, if any
    if (!INIT.empty())
    {
  #ifdef USE_OSG
      BOOST_FOREACH(init_t i, INIT)
      (*i)(MAIN_GROUP, READ_MAP, STEP_SIZE);
  #else
      BOOST_FOREACH(init_t i, INIT)
      (*i)(NULL, READ_MAP, STEP_SIZE);
  #endif
    }
    
    // look for a scene description file
  #ifdef USE_OSG
    if (scene_path != "")
    {
      std::ifstream in(scene_path.c_str());
      if (in.fail())
      {
        std::cerr << "driver: unable to find scene description from " << scene_path << std::endl;
        add_lights();
      }
      else
      {
        in.close();
        osg::Node* node = osgDB::readNodeFile(scene_path);
        if (!node)
        {
          std::cerr << "driver: unable to open scene description file!" << std::endl;
          add_lights();
        }
        else
          MAIN_GROUP->addChild(node);
      }
    }
    else
      add_lights();
  #endif
    
    // process XML options (if possible)
    if (std::string(argv[argc-1]).find(".xml") != std::string::npos)
      process_xml_options(std::string(argv[argc-1]));
    
    // get the simulator visualization
  #ifdef USE_OSG
    MAIN_GROUP->addChild(s->get_persistent_vdata());
    MAIN_GROUP->addChild(s->get_transient_vdata());
  #endif
    
    // setup the timers
    FIRST_STEP_TIME = get_current_time();
    LAST_STEP_TIME = FIRST_STEP_TIME;
    
    // prepare to render
  #ifdef USE_OSG
    if (ONSCREEN_RENDER)
    {
      viewer.setSceneData(MAIN_GROUP);
      viewer.realize();
    }
  #endif
  }

