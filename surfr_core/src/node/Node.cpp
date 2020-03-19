/*
 *
 *  surfr_core/node/Node.cpp
 * 
 *               Generic Device Node class to load and link data with ROS
 *               ----------------------------------------------------------
 *  Begin Date : November 23, 2016
 *  Revision   : January 31, 2019 (rev 26)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  Disclaimer : THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *               CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *               INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *               MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *               DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *               CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *               SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *               BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *               SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *               INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *               WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *               (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *               OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *               POSSIBILITY OF SUCH DAMAGE.
 *               ----------------------------------------------------------
 */


#include "surfr_core/node/Node.h"

namespace surfr {

Node::Node(ros::NodeHandle& nh, boost::shared_ptr<surfr::Base>& dev) :
_PUBS_BUFFER_SIZE(10),
_SUBS_BUFFER_SIZE(10),
_nh(&nh),
_dev(dev),
_t0(ros::Time(0.0)),
_rate(_dev->hz()),
_is_std_msgs(true),
_tf(NULL),
_origin(NULL),
_tcp(NULL),
_robot(NULL) {
 /**
  * Read the ROS parameters of the nodehandle related to the selected "model" device.
  */
  _setup = _dev->config();

  // Type of ROS topic messages
  if (_setup.find("msg_type") == _setup.end()) {
    SURFR_WARNING("ROS param 'msg_type' not found, using std_msgs package by default.");
  } else {
    _is_std_msgs = (!std::string(_setup.at("msg_type")).compare("std")) ? true : false;
  }
  // Check if this device corresponds to a robot manipulator
  bool isManipulator = false;
  if (_setup.find("is_manipulator") == _setup.end()) {
    SURFR_DEBUG("ROS param 'is_manipulator' not found, set to 'false' by default.");
  } else {
    isManipulator = bool(_setup["is_manipulator"]);
  }
// Origin reference frame (if available)
  bool isOrigin = false;
  for (Params::iterator it = _setup.begin(); it != _setup.end(); it++) {
    if ((it->first).find("origin/") == 0) {
      isOrigin = true;
      break;
    }
  }
// Tool Center Point reference frame (if available)
  bool isTcp = false;
  for (Params::iterator it = _setup.begin(); it != _setup.end(); it++) {
    if ((it->first).find("tcp/") == 0) {
      isTcp = true;
      break;
    }
  }

 /**
  * Initialization of the Origin Reference Frame (if exists).
  */
  std::string    origin_parent, origin_child;
  tf::Vector3    P(0.0, 0.0, 0.0);
  tf::Quaternion R(0.0, 0.0, 0.0, 1.0);
  if (!isOrigin) {
    SURFR_DEBUG("Origin parameter not configured, keeping unset.");
    // origin_parent = "world";
    // origin_child  = _dev->name() + "/root_link";
    // SURFR_DEBUG("Origin parameter not configured, creating a default transform at " + origin_child + ".");
  } else {
    if (_setup.find("origin/child") == _setup.end()) {
      SURFR_DEBUG("ROS param 'origin/child' not found, assuming 'root_link' as child.");
      origin_child = _dev->name() + "/root_link";
    } else {
      origin_child = std::string(_setup.at("origin/child"));
    }
    if (_setup.find("origin/parent") == _setup.end()) {
      SURFR_DEBUG("ROS param 'origin/parent' not found, assuming 'world' as parent.");
      origin_parent = "world";
    } else {
      origin_parent = std::string(_setup.at("origin/parent"));
    }
    if (_setup.find("origin/position") == _setup.end()) {
      SURFR_DEBUG("ROS param 'origin/position' not found, assuming null translation.");
    } else {
      std::vector<double_t> P_yaml = surfr::parse_string<double_t>((const std::string)(_setup["origin/position"]));
      P = tf::Vector3(P_yaml[0], P_yaml[1], P_yaml[2]);
    }
    if (_setup.find("origin/orientation") == _setup.end()) {
      SURFR_DEBUG("ROS param 'origin/orientation' not found, assuming null rotation.");
    } else {
      std::vector<double_t> R_yaml = surfr::parse_string<double_t>((const std::string)(_setup["origin/orientation"]));
        Eigen::Quaterniond Q_yaml = Eigen::AngleAxisd(R_yaml[0], Eigen::Vector3d::UnitX())
                                   *Eigen::AngleAxisd(R_yaml[1], Eigen::Vector3d::UnitY())
                                   *Eigen::AngleAxisd(R_yaml[2], Eigen::Vector3d::UnitZ());
      R = tf::Quaternion(Q_yaml.x(), Q_yaml.y(), Q_yaml.z(), Q_yaml.w()); 
  //   }
  // }
      SURFR_DEBUG("Origin child frame : " << origin_child);
      SURFR_DEBUG("Origin parent frame: " << origin_parent);
      SURFR_DEBUG("Origin Position    : [" << P.x() << ", " << P.y() << ", " << P.z() << "]");
      SURFR_DEBUG("Origin Orientation : [" << R.x() << ", " << R.y() << ", " << R.z() << "| " << R.w() << "]");
      _origin = new tf::StampedTransform(tf::Transform(R, P),
                                         _t0,
                                         origin_parent,
                                         origin_child);
    }
  }

 /**
  * Initialization of the Tool Center Point Reference Frame (if exists).
  */
  std::string    tcp_parent, tcp_child;
  tf::Vector3    P_tcp(0.0, 0.0, 0.0);
  tf::Quaternion R_tcp(0.0, 0.0, 0.0, 1.0);
  if (!isTcp) {
    SURFR_DEBUG("TCP parameter not configured, keeping unset.");
    tcp_parent = "world";
    tcp_child  = "world";
  } else {
    if (_setup.find("tcp/child") == _setup.end()) {
      SURFR_DEBUG("ROS param 'tcp/child' not found, assuming 'tcp_link' as child.");
      tcp_child = _dev->name() + "/tcp_link";
    } else {
      tcp_child = std::string(_setup.at("tcp/child"));
    }
    if (_setup.find("tcp/parent") == _setup.end()) {
      SURFR_DEBUG("ROS param 'tcp/parent' not found, assuming 'ee_link' as parent.");
      tcp_parent = _dev->name() + "/ee_link";
    } else {
      tcp_parent = std::string(_setup.at("tcp/parent"));
    }
    if (_setup.find("tcp/position") == _setup.end()) {
      SURFR_DEBUG("ROS param 'tcp/position' not found, assuming null translation.");
    } else {
      std::vector<double_t> P_yaml = surfr::parse_string<double_t>((const std::string)(_setup["tcp/position"]));
      P_tcp = tf::Vector3(P_yaml[0], P_yaml[1], P_yaml[2]);
    }
    if (_setup.find("tcp/orientation") == _setup.end()) {
      SURFR_DEBUG("ROS param 'tcp/orientation' not found, assuming null rotation.");
    } else {
      std::vector<double_t> R_yaml = surfr::parse_string<double_t>((const std::string)(_setup["tcp/orientation"]));
        Eigen::Quaterniond Q_yaml = Eigen::AngleAxisd(R_yaml[0], Eigen::Vector3d::UnitX())
                                   *Eigen::AngleAxisd(R_yaml[1], Eigen::Vector3d::UnitY())
                                   *Eigen::AngleAxisd(R_yaml[2], Eigen::Vector3d::UnitZ());
      R_tcp = tf::Quaternion(Q_yaml.x(), Q_yaml.y(), Q_yaml.z(), Q_yaml.w()); 
    }
    SURFR_DEBUG("TCP child frame : " << tcp_child);
    SURFR_DEBUG("TCP parent frame: " << tcp_parent);
    SURFR_DEBUG("TCP Position    : [" << P_tcp.x() << ", " << P_tcp.y() << ", " << P_tcp.z() << "]");
    SURFR_DEBUG("TCP Orientation : [" << R_tcp.x() << ", " << R_tcp.y() << ", " << R_tcp.z() << "| " << R_tcp.w() << "]");
    _tcp = new tf::StampedTransform(tf::Transform(R_tcp, P_tcp),
                                       _t0,
                                       tcp_parent,
                                       tcp_child);
                                    }

 /**
  * Initialization of the MoveIt! Robot Model (if applicable)
  */
  if (isManipulator) {
  // Load the robot model
    _robot = new surfr::dev::Manipulator(_dev->name() + "/urdf/model");
_robot->control(true);
  // A robot always require an available Origin Reference Frame.
    if (!isOrigin) {
      isOrigin = true;
      _origin  = new tf::StampedTransform(tf::Transform(R, P),
                                          _t0,
                                          "world",
                                          _robot->links_name()[0]);
    }
  // Setup of frames related to each link of the robot
    std::string parent, child;
    std::vector<std::string> links = _robot->links_name();
    for (std::vector<std::string>::iterator it = links.begin(); it != links.end(); it++) {
      parent = (it == links.begin()) ? _robot->frame() : *(it-1);
      child  = *it;
      ROS_INFO_STREAM("Parent of Link " << child << ": " << parent);
      _frames[child] = tf::StampedTransform(tf::Transform(),
                                            _t0,
                                            _robot->frame(),
                                            child);
    }
  // Publish general URDF info with rosparam
    _nh->setParam("moveit/dof"   , _robot->dof());
    _nh->setParam("moveit/joints", _robot->joints_name());
  // Add new publishers and subscribers related to MoveIt! Library
    _setup["moveit/outputs/joints"]   = "Joint";
    _setup["moveit/outputs/pose"]     = "Vector";
    _setup["moveit/outputs/jacobian"] = "Matrix";
    _setup["moveit/outputs/ik"]       = "Joint";
    _setup["moveit/inputs/joints"]    = "Joint";
    _setup["moveit/inputs/pose"]      = "Vector";
    _setup["moveit/inputs/ik"]        = "Vector";
    if (_dev->is_simulation()) {
      _setup["outputs/joints"] = "Joint";
      _setup["outputs/pose"]   = "Vector";
    }
  }

 /**
  * Initialization of the TransForm Topic BroadCaster (if applicable)
  */
  if (_origin) {
    _tf = new tf::TransformBroadcaster;
  }

  this->_deploy_topics();
  this->_deploy_services();
  ros::Rate warm(1);
  ROS_INFO("Warming device data flow...");
  warm.sleep();
  ROS_INFO("Done!");
}

Node::~Node() {
  // IMPORTANT NOTE: As _dev points to an object created outside DeviceNode class,
  //                 that code is responsible of its deletion.
  if (_robot) {
    delete _robot;
  }
  if (_tf) {
    delete _tf;
  }
  if (_origin) {
    delete _origin;
  }
  for (Filters::iterator it = _filters.begin(); it != _filters.end(); it++) {
    delete it->second;
  }
  for (Planners::iterator it = _plans.begin(); it != _plans.end(); it++) {
    delete it->second;
  }
}

void Node::update() {
  //! The sleep at the beginning of the loop allows the device to load a valid first measure.
  _rate.sleep();

  this->_publish_topics();
}

void Node::_deploy_topics() {
  bool  isValidType = true, isFilter = false, isPlan = false;
  const std::string root_out    = "outputs";
  const std::string root_in     = "inputs";
  const std::string root_filter = "filters";
  const std::string root_plan   = "planners";
        std::string topic, type, root, key;
// BEGIN: This section is for filters
  XmlRpc::XmlRpcValue filter_list, filter_info;
    _nh->getParam(root_filter            , filter_list);
    _nh->getParam("/surfr/" + root_filter, filter_info);
  surfr::Params filter_params;
// END  : This section is for filters
// BEGIN: This section is for planners
  XmlRpc::XmlRpcValue plan_list, plan_info;
    _nh->getParam(root_plan            , plan_list);
    _nh->getParam("/surfr/" + root_plan, plan_info);
  surfr::Params plan_params;
// END  : This section is for planners


  for (Params::iterator it = _setup.begin(); it != _setup.end(); it++) {
    if ((((it->first).find(root_in)     == std::string::npos) &&
         ((it->first).find(root_out)    == std::string::npos)) ||
        (((it->first).find(root_filter) != std::string::npos) &&
         (_setup.find((it->first).substr(root_filter.size() + 1)) == _setup.end())) ||
        (((it->first).find(root_plan) != std::string::npos) &&
         (_setup.find((it->first).substr(root_plan.size() + 1)) == _setup.end())) ||
        ((it->second).getType() != XmlRpc::XmlRpcValue::TypeString)) {
      continue;
    }
    topic    = it->first;
    key      = topic.substr(topic.rfind("/") + 1);
    isFilter = topic.find(root_filter) != std::string::npos;
    isPlan   = topic.find(root_plan) != std::string::npos;
    root     = topic.substr(isFilter*(root_filter.size() + 1) + isPlan*(root_plan.size() + 1),
                            topic.rfind("/") - isFilter*(root_filter.size() + 1) - isPlan*(root_plan.size() + 1));
// std::cout << "Topic   : " << topic << std::endl;
// std::cout << "Root    : " << root << std::endl;
// std::cout << "Key     : " << key << std::endl;
// std::cout << "isFilter: " << isFilter << std::endl;
// std::cout << "isPlan  : " << isPlan << std::endl;

    // Check if these output data has to be filtered
    filter_params.clear();
    if (isFilter) {
      type = std::string(_setup[root + "/" + key]);
      std::string filter_name    = std::string(filter_list[root][key]);
      filter_params[filter_name] = filter_info[filter_name];
// std::cout << "Type    : " << type << std::endl;
// std::cout << "filter_list: " << filter_list << std::endl;
// std::cout << "filter_name: " << filter_name << std::endl;
// std::cout << "filter_info: " << filter_info << std::endl;
// std::cout << "filter_params: " << filter_params[filter_name] << std::endl;
    // Check if these input data has a planner
    } else if (isPlan) {
      type = std::string(_setup[root + "/" + key]);
      std::string plan_name  = std::string(plan_list[root][key]);
      plan_info[plan_name]["dt"] = _dev->dt();
      plan_params[plan_name] = plan_info[plan_name];
std::cout << "Type    : " << type << std::endl;
std::cout << "plan_list: " << plan_list << std::endl;
std::cout << "plan_name: " << plan_name << std::endl;
std::cout << "plan_info: " << plan_info << std::endl;
std::cout << "plan_params: " << plan_params[plan_name] << std::endl;
    } else {
      type = std::string(it->second);
    }

    if (topic.find(root_out) != std::string::npos) {
      // NOTE: The valid geometry_msgs messages can be used with both, the ROS
      //       standard and SurFr custom messages. These messages are intended
      //       to be linked with the RVIZ virtual environment by default.
      if        (!type.compare("Point" )) {
        _pubs[topic] = _nh->advertise<geometry_msgs::Point>    (topic, _PUBS_BUFFER_SIZE);
      } else if (!type.compare("Pose")) {
        _pubs[topic] = _nh->advertise<geometry_msgs::Pose>     (topic, _PUBS_BUFFER_SIZE);
      } else if (!type.compare("Transform")) {
        _pubs[topic] = _nh->advertise<geometry_msgs::Transform>(topic, _PUBS_BUFFER_SIZE);
      } else if (!type.compare("Twist")) {
        _pubs[topic] = _nh->advertise<geometry_msgs::Twist>    (topic, _PUBS_BUFFER_SIZE);
      } else if (!type.compare("Wrench")) {
        _pubs[topic] = _nh->advertise<geometry_msgs::Wrench>   (topic, _PUBS_BUFFER_SIZE);
      }
      if (_pubs.find(topic) != _pubs.end()) {
        continue;
      }

     /**
      * The advertise() function is how you tell ROS that you want to
      * publish on a given topic name. This invokes a call to the ROS
      * master node, which keeps a registry of who is publishing and who
      * is subscribing. After this advertise() call is made, the master
      * node will notify anyone who is trying to subscribe to this topic name,
      * and they will in turn negotiate a peer-to-peer connection with this
      * node.  advertise() returns a Publisher object which allows you to
      * publish messages on that topic through a call to publish().  Once
      * all copies of the returned Publisher object are destroyed, the topic
      * will be automatically unadvertised.
      *
      * The second parameter to advertise() is the size of the message queue
      * used for publishing messages.  If messages are published more quickly
      * than we can send them, the number here specifies how many messages to
      * buffer up before throwing some away.
      */
      if (_is_std_msgs) {
        if        (!type.compare("Boolean" )) {
          _pubs[topic] = _nh->advertise<std_msgs::Bool>   (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Unsigned")) {
          _pubs[topic] = _nh->advertise<std_msgs::UInt32> (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Integer" )) {
          _pubs[topic] = _nh->advertise<std_msgs::Int64>  (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Double"  )) {
          _pubs[topic] = _nh->advertise<std_msgs::Float64>(topic, _PUBS_BUFFER_SIZE);
          if (isFilter) {
            _filters[root_out + "/" + key]     = new surfr::filters::Filter(filter_params);
            ROS_DEBUG_STREAM("Output " << topic << " publishes filtered data from " << root_out + "/" + key);
          }
        } else if (!type.compare("String"  )) {
          _pubs[topic] = _nh->advertise<std_msgs::String> (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Vector" )) {
          _pubs[topic] = _nh->advertise<std_msgs::Float64MultiArray>(topic, _PUBS_BUFFER_SIZE);
          if (isFilter) {
            _filters[root_out + "/" + key]     = new surfr::filters::Filter(filter_params);
            ROS_DEBUG_STREAM("Output " << topic << " publishes filtered data from " << root_out + "/" + key);
          }
        } else if (!type.compare("Matrix" )) {
          _pubs[topic] = _nh->advertise<std_msgs::Float64MultiArray>(topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Joint"   )) {
          _pubs[topic] = _nh->advertise<sensor_msgs::JointState>(topic, _PUBS_BUFFER_SIZE);
          // if (isFilter) {
          //   _filters[topic + "/position"]    = new surfr::filters::Filter(filter_params);
          //   _filters[topic + "/velocity"]    = new surfr::filters::Filter(filter_params);
          //   _filters[topic + "/effort"]      = new surfr::filters::Filter(filter_params);
          //   _pubs[topic_filter] = _nh->advertise<sensor_msgs::JointState>(topic_filter, 1); 
          //   ROS_DEBUG_STREAM("Publisher Filter " << topic_filter << " (" << type << ")");
          // }
        } else {
          isValidType = false;
        }
      } else {
        if        (!type.compare("Boolean" )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Boolean> (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Unsigned")) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Unsigned>(topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Integer" )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Integer> (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Double"  )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Double>  (topic, _PUBS_BUFFER_SIZE);
          if (isFilter) {
            _filters[root_out + "/" + key] = new surfr::filters::Filter(filter_params);
            ROS_DEBUG_STREAM("Output " << topic << " publishes filtered data from " << root_out + "/" + key);
          }
        } else if (!type.compare("String"  )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::String>  (topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Vector" )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Vector>  (topic, _PUBS_BUFFER_SIZE);
          if (isFilter) {
            _filters[root_out + "/" + key] = new surfr::filters::Filter(filter_params);
            ROS_DEBUG_STREAM("Output " << topic << " publishes filtered data from " << root_out + "/" + key);
          }
        } else if (!type.compare("Matrix" )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Matrix>(topic, _PUBS_BUFFER_SIZE);
        } else if (!type.compare("Joint"   )) {
          _pubs[topic] = _nh->advertise<surfr_msgs::Joint>   (topic, _PUBS_BUFFER_SIZE);
          // if (isFilter) {
          //   _filters[topic + "/position"]    = new surfr::filters::Filter(filter_params);
          //   _filters[topic + "/velocity"]    = new surfr::filters::Filter(filter_params);
          //   _filters[topic + "/effort"]      = new surfr::filters::Filter(filter_params);
          //   _filters[topic + "/current"]     = new surfr::filters::Filter(filter_params);
          //   _filters[topic + "/temperature"] = new surfr::filters::Filter(filter_params);
          //   _pubs[topic_filter] = _nh->advertise<surfr_msgs::Joint>(topic_filter, 1); 
          //   ROS_DEBUG_STREAM("Publisher Filter " << topic_filter << " (" << type << ")");
          // }
        } else {
          isValidType = false;
        }
      }
      ROS_ASSERT(isValidType);
      ROS_DEBUG_STREAM("Publisher " << topic << " (" << type << ")");
    }

    if (topic.find(root_in) != std::string::npos) {
      if (_is_std_msgs) {
        if        (!type.compare("Boolean" )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::Bool>   , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Unsigned")) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::UInt32> , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Integer" )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::Int64>  , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Double"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::Float64>, this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("String"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::String> , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Vector"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::Float64MultiArray>, this,
                                        ros::TransportHints().tcpNoDelay());
          if (isPlan) {
            _plans[root_in + "/" + key] = new surfr::planners::Planner(plan_params);
            ROS_DEBUG_STREAM("Input " << topic << " plans received data from " << root_in + "/" + key);
          }
        } else if (!type.compare("Matrix"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<std_msgs::Float64MultiArray>, this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Joint"   )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<sensor_msgs::JointState>, this,
                                        ros::TransportHints().tcpNoDelay());
        } else {
          isValidType = false;
        }
      } else {
        if        (!type.compare("Boolean" )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Boolean> , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Unsigned")) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Unsigned>, this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Integer" )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Integer> , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Double"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Double>  , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("String"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::String>  , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Vector"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Vector>  , this,
                                        ros::TransportHints().tcpNoDelay());
          if (isPlan) {
            _plans[root_in + "/" + key] = new surfr::planners::Planner(plan_params);
            ROS_DEBUG_STREAM("Input " << topic << " plans received data from " << root_in + "/" + key);
          }
        } else if (!type.compare("Matrix"  )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Matrix>  , this,
                                        ros::TransportHints().tcpNoDelay());
        } else if (!type.compare("Joint"   )) {
          _subs[topic] = _nh->subscribe(topic, _SUBS_BUFFER_SIZE, &Node::_cbSubscriber<surfr_msgs::Joint>   , this,
                                        ros::TransportHints().tcpNoDelay());
        } else { 
          isValidType = false;
        }
      }
      ROS_ASSERT(isValidType);
      ROS_DEBUG_STREAM("Subscriber " << topic << " (" << type << ")");
    }
  }
}

void Node::_deploy_services() {
  for(Params::iterator it = _setup.begin(); it != _setup.end(); it++) {
    // A service tag has been found
    bool isFoundSrv = ((it->first).find("services") != std::string::npos) ? true : false; 
    // Check if sub-tag is the service type
    if (isFoundSrv && ((it->first).find("type") != std::string::npos)) {
      // If so, create the new service with the specified type
      const std::string srvName = (it->first).substr((it->first).find("/")+1, (it->first).rfind("/")-(it->first).find("/")-1);
      if (boost::iequals("empty", std::string(it->second))) {
        _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<surfr_msgs::cmd_empty::Request,
                                                                         surfr_msgs::cmd_empty::Response>   , this);
      } else if (boost::iequals("boolean", std::string(it->second))) {
        _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<surfr_msgs::cmd_boolean::Request,
                                                                         surfr_msgs::cmd_boolean::Response> , this);
      // } else if (boost::iequals("unsigned", std::string(it->second))) {
      //   _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<int, surfr_msgs::cmd_unsigned::Request>, this);
      } else if (boost::iequals("integer", std::string(it->second))) {
        _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<surfr_msgs::cmd_integer::Request,
                                                                         surfr_msgs::cmd_integer::Response> , this);
      } else if (boost::iequals("double", std::string(it->second))) {
        _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<surfr_msgs::cmd_double::Request,
                                                                         surfr_msgs::cmd_double::Response>  , this);
      } else if (boost::iequals("string", std::string(it->second))) {
        _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<surfr_msgs::cmd_string::Request,
                                                                         surfr_msgs::cmd_string::Response>  , this);
      } else if (boost::iequals("vector", std::string(it->second))) {
        _srv[srvName] = _nh->advertiseService(srvName, &Node::_cbService<surfr_msgs::cmd_vector::Request,
                                                                         surfr_msgs::cmd_vector::Response>  , this);
      }
      if (_srv.find(srvName) != _srv.end()) {
        ROS_DEBUG_STREAM("Service (" << it->second << ") " << srvName);
      }
    }
  }
}

void Node::_publish_topics() {
         bool          isValidType = true, isFilter = false;
  const  std::string   root = "outputs";
         std::string   topic, key, type;
         ros::V_string list;
  // const  ros::Time     t = ros::Time((ros::Time::now() - _t0).toSec());
         ros::Time     t = ros::Time((ros::Time::now() - _t0).toSec());
  // This must be static to avoid memory overflow
  static Flow out;

 /**
  * Read the Output Data Flow (from simulation or real device)
  */
  if (_dev->is_simulation()) {
    if (_robot) {
      (out)["joints/name"       ] = _robot->joints_name();
      (out)["joints/position"   ] = _robot->joints_position();
      //~ (out)["joints/velocity"   ] = _robot->joints_velocity();
      (out)["joints/effort"     ] = std::vector<double_t>(_robot->dof());
      (out)["pose"              ] = _robot->pose(surfr::dev::LOCAL_FRAME);
      if (!_is_std_msgs) {
        (out)["joints/state"      ] = std::vector<int>(_robot->dof());
        (out)["joints/current"    ] = std::vector<double_t>(_robot->dof());
        (out)["joints/temperature"] = std::vector<double_t>(_robot->dof());
      }
    }
  } else {
    _dev->output(out);
    // MoveIt! update in case of a robot manipulator device
    if (_robot) {
      (out)["joints/name"       ] = _robot->joints_name();
      std::vector<double> Q = FLOW_CAST(out, std::vector<double>, "joints/position");
      _robot->update(Q);
      // if (_robot->control()) {
      //   (out)["pose"] = _robot->pose();
      // }
    }
  }

 /**
  * Send the Input Data Flow when planning or in simulation mode
  */
  // if (_in.size() > 0) {
  if ((_dev->is_simulation()) && (_in.size() > 0)) {
// NOTE: THIS MUST DETECT A GENERIC INPUT (NOW ONLY FOR ROBOT INPUT JOINTS)
    if (_in.find("topic") != _in.end()) {
      std::string cmd = FLOW_CAST(_in, std::string, "topic");
      if (cmd.find("move_joints") != std::string::npos) {
        std::vector<double> Q = FLOW_CAST(_in, std::vector<double>, "data/position");
        _robot->update(Q);
      }
      // NOTE: speed_pose sends topic to speed command of the robot
      //       Planner process() method computes the next velocity for the robot
      //       from a target position
      if (cmd.find("speed_pose") != std::string::npos) {
        if (cmd.find("planners") != std::string::npos) {
          std::vector<double_t> target   = FLOW_CAST(_in, std::vector<double>, "plan"),
                                current  = FLOW_CAST(out, std::vector<double>, "pose"),
                                velocity = FLOW_CAST(out, std::vector<double>, "velocity");
          _in["data"]  = _plans["inputs/speed_pose"]->process(target, current, velocity);
          _dev->input(_in);
        }
      }
    }
  }

 /**
  * Update of the TransForm Topic BroadCaster (if applicable)
  */
  if (_tf) {
  // Reference Frame of Origin
// ***** BEGIN: Prevents blinking of the 3D robot model visualization *****
std::vector<tf::StampedTransform> tfList;
t = ros::Time::now();
// ***** END **************************************************************
    if (_origin) {
      // std::string origin_position;
      //   _nh->getParamCached("origin/position", origin_position);
      //   std::vector<double_t> P_yaml = surfr::parse_string<double_t>((const std::string)(origin_position));
      //   tf::Vector3 P(P_yaml[0], P_yaml[1], P_yaml[2]);
      // std::string origin_orientation;
      //   _nh->getParamCached("origin/orientation", origin_orientation);
      //   std::vector<double_t> R_yaml = surfr::parse_string<double_t>((const std::string)(origin_orientation));
      //   Eigen::Quaterniond Q_yaml = Eigen::AngleAxisd(R_yaml[0], Eigen::Vector3d::UnitX())
      //                              *Eigen::AngleAxisd(R_yaml[1], Eigen::Vector3d::UnitY())
      //                              *Eigen::AngleAxisd(R_yaml[2], Eigen::Vector3d::UnitZ());
      //   tf::Quaternion Q(Q_yaml.x(), Q_yaml.y(), Q_yaml.z(), Q_yaml.w()); 
      // _origin->setOrigin(P);
      // _origin->setRotation(Q);

      _origin->stamp_ = t;
      // _tf->sendTransform(*_origin);
tfList.push_back(*_origin);
    }
  // Reference Frame of Robot Links
    if (_robot) {
      ros::V_string        links = _robot->links_name();
      Eigen::Affine3d      T;
      for (ros::V_string::iterator it = links.begin(); it != links.end(); it++) {
// if (*it != (_dev->name() + "/root_link")) {
        T = _robot->transform(*it);
        Eigen::Vector3d    P(T.translation());
        Eigen::Quaterniond Q(T.rotation());
        _frames[*it].stamp_ = t;
        _frames[*it].setData(tf::Transform(tf::Quaternion(Q.x(), Q.y(), Q.z(), Q.w()),
                                           tf::Vector3(P.x(), P.y(), P.z())));
        // _tf->sendTransform(_frames[*it]);
tfList.push_back(_frames[*it]);
// }
      }
    }
  // Reference Frame of TCP
    if (_tcp) {
      _tcp->stamp_ = t;
      // _tf->sendTransform(*_tcp);
tfList.push_back(*_tcp);
    }
_tf->sendTransform(tfList);
  }

 /**
  * Update of the MoveIt! topics information (if applicable)
  */
  if (_robot) {
// FALTA LA OPCION DE MENSAJES DEL SURGICAL FRAMEWORK
  // MoveIt! joints state
    sensor_msgs::JointState msg_joints;
      msg_joints.header.stamp = t;
      msg_joints.name         = _robot->joints_name();
      msg_joints.position     = _robot->joints_position();
      //~ msg_joints.velocity     = _robot->joints_velocity();
    _pubs["moveit/outputs/joints"].publish(msg_joints);
  // MoveIt! pose
    std_msgs::Float64MultiArray msg_pose;
      msg_pose.data = _robot->pose();
    _pubs["moveit/outputs/pose"].publish(msg_pose);
  // MoveIt! Inverse Kinematics
    if (_in.size()) {
      if (_in.find("topic") != _in.end()) {
        std::string topic = FLOW_CAST(_in, std::string, "topic");
        if (topic.find("ik") != std::string::npos) {
          std::vector<double_t> pose = FLOW_CAST(_in, std::vector<double_t>, "data");
          msg_joints.position   = _robot->joints_ik(pose);
          msg_joints.velocity   = std::vector<double_t>(_robot->dof(), 0.0);
          _pubs["moveit/outputs/ik"].publish(msg_joints);
        }
      }
    }
  // MoveIt! jacobian
    // Jacobian message layout (dimension of pose(6)xjoints(dof))
    std_msgs::MultiArrayLayout layout;
      std_msgs::MultiArrayDimension dim;
        dim.label  = "joints";
        dim.size   = _robot->dof();
        dim.stride = _robot->dof();
      layout.dim.push_back(dim);
        dim.label  = "pose";
        dim.size   = 6;
        dim.stride = 6*_robot->dof();
      layout.dim.push_back(dim);
      layout.data_offset = 0;
    Eigen::MatrixXd J = _robot->jacobian();
    if (_is_std_msgs) {
      std_msgs::Float64MultiArray msg_jacobian;
        msg_jacobian.layout = layout;
        for (uint16_t rows = 0; rows < 6; rows++) {
          for (uint16_t cols = 0; cols < _robot->dof(); cols++) {
            msg_jacobian.data.push_back(J(rows, cols));
          }
        }
      _pubs["moveit/outputs/jacobian"].publish(msg_jacobian);
    } else {
      surfr_msgs::Matrix msg_jacobian;
        msg_jacobian.layout = layout;
        for (uint16_t rows = 0; rows < 6; rows++) {
          for (uint16_t cols = 0; cols < _robot->dof(); cols++) {
            msg_jacobian.data.push_back(J(rows, cols));
          }
        }
      _pubs["moveit/outputs/jacobian"].publish(msg_jacobian);
    }
  }

 /**
  * Update only data received from device or simulator
  */
  for (Flow::iterator it = out.begin(); it != out.end(); it++) {
      key      = (it->first).substr(0, (it->first).find('/'));
        // Correct behavior of Joint messages
        if (std::find(list.begin(), list.end(), key) != list.end()) {
          continue;
        } else {
          list.push_back(key);
        }
      topic    = root + "/" + key;
      type     = std::string(_setup[topic]);
      isFilter = (_filters.find(topic) != _filters.end());

      if (_is_std_msgs) {
        if        (!type.compare("Boolean")) {
          std_msgs::Bool msg;
            msg.data        = FLOW_CAST(out, bool, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Unsigned")) {
          std_msgs::UInt32 msg;
            msg.data        = FLOW_CAST(out, unsigned int, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Integer")) {
          std_msgs::Int64 msg;
            msg.data        = FLOW_CAST(out, long, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Double")) {
          std_msgs::Float64 msg;
            msg.data        = FLOW_CAST(out, double, it->first);
          _pubs[topic].publish(msg);
          if (isFilter) {
            msg.data        = _filters[topic]->process(msg.data);
            _pubs["filters/" + topic].publish(msg);
          }
        } else if (!type.compare("String")) {
          std_msgs::String msg;
            msg.data        = FLOW_CAST(out, std::string, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Vector")) {
          std_msgs::Float64MultiArray msg;
            msg.data        = FLOW_CAST(out, std::vector<double>, it->first);
          _pubs[topic].publish(msg);
          if (isFilter) {
            msg.data        = _filters[topic]->process(msg.data);
            _pubs["filters/" + topic].publish(msg);
          }
        } else if (!type.compare("Matrix")) {
          std_msgs::Float64MultiArray msg;
            msg.data        = FLOW_CAST(out, std::vector<double>, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Joint")) {
          sensor_msgs::JointState msg;
            msg.header.stamp = t;
            msg.name         = FLOW_CAST(out, std::vector<std::string>, key + "/name"       );
            msg.position     = FLOW_CAST(out, std::vector<double>     , key + "/position"   );
            msg.velocity     = FLOW_CAST(out, std::vector<double>     , key + "/velocity"   );
            msg.effort       = FLOW_CAST(out, std::vector<double>     , key + "/effort"     );
          _pubs[topic].publish(msg);
        } else if (!type.compare("Point")) {
          geometry_msgs::Point msg;
            msg.x = FLOW_CAST(out, double, key + "/x");
            msg.y = FLOW_CAST(out, double, key + "/y");
            msg.z = FLOW_CAST(out, double, key + "/z");
          _pubs[topic].publish(msg);
        } else if (!type.compare("Quaternion")) {
          geometry_msgs::Quaternion msg;
            msg.w = FLOW_CAST(out, double, key + "/w");
            msg.x = FLOW_CAST(out, double, key + "/x");
            msg.y = FLOW_CAST(out, double, key + "/y");
            msg.z = FLOW_CAST(out, double, key + "/z");
          _pubs[topic].publish(msg);
        } else if (!type.compare("Pose")) {
          geometry_msgs::Pose msg;
            msg.position.x    = FLOW_CAST(out, double, key + "/position/x"   );
            msg.position.y    = FLOW_CAST(out, double, key + "/position/y"   );
            msg.position.z    = FLOW_CAST(out, double, key + "/position/z"   );
            msg.orientation.w = FLOW_CAST(out, double, key + "/orientation/w");
            msg.orientation.x = FLOW_CAST(out, double, key + "/orientation/y");
            msg.orientation.y = FLOW_CAST(out, double, key + "/orientation/x");
            msg.orientation.z = FLOW_CAST(out, double, key + "/orientation/z");
          _pubs[topic].publish(msg);
        } else if (!type.compare("Transform")) {
          geometry_msgs::Transform msg;
            msg.translation.x = FLOW_CAST(out, double, key + "/translation/x");
            msg.translation.y = FLOW_CAST(out, double, key + "/translation/y");
            msg.translation.z = FLOW_CAST(out, double, key + "/translation/z");
            msg.rotation.w    = FLOW_CAST(out, double, key + "/rotation/w"   );
            msg.rotation.x    = FLOW_CAST(out, double, key + "/rotation/y"   );
            msg.rotation.y    = FLOW_CAST(out, double, key + "/rotation/x"   );
            msg.rotation.z    = FLOW_CAST(out, double, key + "/rotation/z"   );
          _pubs[topic].publish(msg);
        } else if (!type.compare("Twist")) {
          geometry_msgs::Twist msg;
            msg.linear.x  = FLOW_CAST(out, double, key + "/linear/x" );
            msg.linear.y  = FLOW_CAST(out, double, key + "/linear/y" );
            msg.linear.z  = FLOW_CAST(out, double, key + "/linear/z" );
            msg.angular.x = FLOW_CAST(out, double, key + "/angular/y");
            msg.angular.y = FLOW_CAST(out, double, key + "/angular/x");
            msg.angular.z = FLOW_CAST(out, double, key + "/angular/z");
          _pubs[topic].publish(msg);
        } else if (!type.compare("Wrench")) {
          geometry_msgs::Wrench msg;
            msg.force.x  = FLOW_CAST(out, double, key + "/force/x" );
            msg.force.y  = FLOW_CAST(out, double, key + "/force/y" );
            msg.force.z  = FLOW_CAST(out, double, key + "/force/z" );
            msg.torque.x = FLOW_CAST(out, double, key + "/torque/y");
            msg.torque.y = FLOW_CAST(out, double, key + "/torque/x");
            msg.torque.z = FLOW_CAST(out, double, key + "/torque/z");
          _pubs[topic].publish(msg);
        } else {
          isValidType = false;
        }
      } else {
        if        (!type.compare("Boolean")) {
          surfr_msgs::Boolean msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, bool, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Unsigned")) {
          surfr_msgs::Unsigned msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, unsigned int, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Integer")) {
          surfr_msgs::Integer msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, long, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Double")) {
          surfr_msgs::Double msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, double, it->first);
          _pubs[topic].publish(msg);
          if (isFilter) {
            msg.data        = _filters[topic]->process(msg.data);
            _pubs["filters/" + topic].publish(msg);
          }
        } else if (!type.compare("String")) {
          surfr_msgs::String msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, std::string, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Vector")) {
          surfr_msgs::Vector msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, std::vector<double>, it->first);
          _pubs[topic].publish(msg);
          if (isFilter) {
            msg.data        = _filters[topic]->process(msg.data);
            _pubs["filters/" + topic].publish(msg);
          }
        } else if (!type.compare("Matrix")) {
// NOTE: Dimensions are not covered yet (must be taken from 'out' key value)
          surfr_msgs::Matrix msg;
            msg.header.stamp = t;
            msg.data        = FLOW_CAST(out, std::vector<double>, it->first);
          _pubs[topic].publish(msg);
        } else if (!type.compare("Joint")) {
          surfr_msgs::Joint msg;
            msg.header.stamp = t;
            msg.name         = FLOW_CAST(out, std::vector<std::string>, key + "/name"       );
            msg.state        = FLOW_CAST(out, std::vector<int>        , key + "/state"      );
            msg.position     = FLOW_CAST(out, std::vector<double>     , key + "/position"   );
            msg.velocity     = FLOW_CAST(out, std::vector<double>     , key + "/velocity"   );
            msg.current      = FLOW_CAST(out, std::vector<double>     , key + "/current"    );
            msg.effort       = FLOW_CAST(out, std::vector<double>     , key + "/effort"     );
            msg.temperature  = FLOW_CAST(out, std::vector<double>     , key + "/temperature");
          _pubs[topic].publish(msg);
        } else {
          isValidType = false;
        }
      }
      ROS_ASSERT(isValidType);
    }
}

template <class T>
void Node::_cbSubscriber(const ros::MessageEvent<T const>& event) {
  //! If needed, these are the publisher source and the ROS message data type
  // const std::string&      pub    = event.getPublisherName();
  boost::shared_ptr<const T> msg    = event.getMessage();
  ros::Time                  time   = event.getReceiptTime();
  ros::M_string&             header = event.getConnectionHeader();
  std::string                topic  = header.at("topic").substr(header.at("topic").rfind('/')+1);
  std::string                type   = header.at("type");

  _in["topic"] = topic;
  _in["type"]  = type;
  _in["time"]  = (time - _t0).toSec();
  _in["data"]  = msg->data;
  if (topic.find("planners") != std::string::npos) {
    _in["plan"] = msg->data;
  }
  if ((!_dev->is_simulation()) && (topic.find("planners") == std::string::npos)) {
    _dev->input(_in);
  }
  ROS_DEBUG_STREAM("Subscriber received data from topic " << topic << " (" << type << ")");
}

template <>
void Node::_cbSubscriber(const ros::MessageEvent<sensor_msgs::JointState const>& event) {
  //! If needed, these are the publisher source and the ROS message data type
  // const std::string&                            pub    = event.getPublisherName();
  boost::shared_ptr<const sensor_msgs::JointState> msg    = event.getMessage();
  ros::Time                                        time   = event.getReceiptTime();
  ros::M_string&                                   header = event.getConnectionHeader();
  std::string                                      topic  = header.at("topic").substr(header.at("topic").rfind('/')+1);
  std::string                                      type   = header.at("type");

  _in["topic"] = topic;
  _in["type"]  = type;
  _in["time"]  = (time - _t0).toSec();
  _in["data/name"]     = msg->name;
  _in["data/position"] = msg->position;
  _in["data/velocity"] = msg->velocity;
  _in["data/effort"]   = msg->effort;
  if (topic.find("planners") != std::string::npos) {
    _in["plan/position"] = msg->position;
    _in["plan/velocity"] = msg->velocity;
    _in["plan/effort"]   = msg->effort;
  }
  if ((!_dev->is_simulation()) && (topic.find("planners") == std::string::npos)) {
    _dev->input(_in);
  }

  ROS_DEBUG_STREAM("Subscriber received data from topic " << header.at("topic") << " (" << type << "): ");
  if (msg->name.size())     ROS_DEBUG_STREAM("  Name    : " << (FLOW_CAST(_in, std::vector<std::string>, "data/name")));
  if (msg->position.size()) ROS_DEBUG_STREAM("  Position: " << (FLOW_CAST(_in, std::vector<double>, "data/position")));
  if (msg->velocity.size()) ROS_DEBUG_STREAM("  Velocity: " << (FLOW_CAST(_in, std::vector<double>, "data/velocity")));
  if (msg->effort.size())   ROS_DEBUG_STREAM("  Effort  : " << (FLOW_CAST(_in, std::vector<double>, "data/effort")));
}

template <>
void Node::_cbSubscriber(const ros::MessageEvent<surfr_msgs::Joint const>& event) {
  //! If needed, these are the publisher source and the ROS message data type
  // const std::string&                      pub    = event.getPublisherName();
  boost::shared_ptr<const surfr_msgs::Joint> msg    = event.getMessage();
  ros::Time                                  time   = event.getReceiptTime();
  ros::M_string&                             header = event.getConnectionHeader();
  std::string                                topic  = header.at("topic").substr(header.at("topic").rfind('/')+1);
  std::string                                type   = header.at("type");

  _in["topic"] = topic;
  _in["type"]  = type;
  _in["time"]  = (time - _t0).toSec();
  _in["data/name"]        = msg->name;
  _in["data/state"]       = msg->state;
  _in["data/position"]    = msg->position;
  _in["data/velocity"]    = msg->velocity;
  _in["data/current"]     = msg->current;
  _in["data/effort"]      = msg->effort;
  _in["data/temperature"] = msg->temperature;
  if (topic.find("planners") != std::string::npos) {
    _in["plan/position"] = msg->position;
    _in["plan/velocity"] = msg->velocity;
    _in["plan/current"]  = msg->current;
    _in["plan/effort"]   = msg->effort;
  }
  if ((!_dev->is_simulation()) && (topic.find("planners") == std::string::npos)) {
    _dev->input(_in);
  }

  ROS_DEBUG_STREAM("Subscriber received data from topic " << header.at("topic") << " (" << type << "): ");
  ROS_DEBUG_STREAM("  Name       : " << (FLOW_CAST(_in, std::vector<std::string>, "data/name")));
  ROS_DEBUG_STREAM("  State      : " << (FLOW_CAST(_in, std::vector<std::string>, "data/state")));
  ROS_DEBUG_STREAM("  Position   : " << (FLOW_CAST(_in, std::vector<double>, "data/position")));
  ROS_DEBUG_STREAM("  Velocity   : " << (FLOW_CAST(_in, std::vector<double>, "data/velocity")));
  ROS_DEBUG_STREAM("  Current    : " << (FLOW_CAST(_in, std::vector<double>, "data/current")));
  ROS_DEBUG_STREAM("  Effort     : " << (FLOW_CAST(_in, std::vector<double>, "data/effort")));
  ROS_DEBUG_STREAM("  Temperature: " << (FLOW_CAST(_in, std::vector<double>, "data/temperature")));
}

template<class Req, class Res>
bool Node::_cbService(ros::ServiceEvent<Req, Res>& event) {
  std::string         name = event.getConnectionHeader().at("service");
  XmlRpc::XmlRpcValue data = XmlRpc::XmlRpcValue(event.getRequest().param);
  
  return _cbServiceBase(name, data);
}

template<>
bool Node::_cbService(ros::ServiceEvent<surfr_msgs::cmd_empty::Request,
                                        surfr_msgs::cmd_empty::Response>& event) {
  std::string         name = event.getConnectionHeader().at("service");
  XmlRpc::XmlRpcValue data = _setup["services/" + name.substr(name.rfind('/') + 1) + "/data"];

  return _cbServiceBase(name, data);
}

template<>
bool Node::_cbService(ros::ServiceEvent<surfr_msgs::cmd_vector::Request,
                                        surfr_msgs::cmd_vector::Response>& event) {
  std::string         name = event.getConnectionHeader().at("service");
  XmlRpc::XmlRpcValue data;
  for (unsigned int k = 0; k < event.getRequest().param.size(); k++) {
    data[k] = event.getRequest().param[k];
  }

  return _cbServiceBase(name, data);
}

bool Node::_cbServiceBase(std::string& name, XmlRpc::XmlRpcValue& data) {
  bool        ret      = true;
  std::string srvName  = name.substr(name.rfind("/") + 1),                                // Service name
              srvTopic,                                                                   // Command requested
              srvPath  = "services/" + srvName,                                           // Service path at YAML file
              srvType  = _setup[srvPath + "/type"];                                       // Always exist (checked at deploy_services())
  ros::Time   srvTime  = ros::Time::now();                                                // Instant when service was called


  // Check: Label 'data' must set into YAML file, at least with the requested command
  ret &= (_setup.find(srvPath + "/input") != _setup.end());
  if (ret) {
    srvTopic = std::string(_setup[srvPath + "/input"]);
  } else {
    ROS_WARN_STREAM("Label '" << srvPath << "/input' must be set for any service type.");
  }

  // Store all parameters to be sent to the device service
  if (ret) {
    try {
      _in.clear();
      _in["service"] = srvName;
      // _in["topic"]   = _dev->name() + "/inputs/" + srvTopic;
      _in["topic"]   = srvTopic;
      _in["type"]    = srvType;
      _in["time"]    = (srvTime - _t0).toSec();
      if (data.valid()) {
        _in["data"]    = data;
      }
    } catch(std::exception const err) {
      SURFR_ERROR(err.what());
      ret = false;
    }

    // Call requested service with its parameter values
    if (!_dev->is_simulation()) {
      ret &= _dev->service(_in);
    }
  }

  return ret;
}

} // namespace surfr
