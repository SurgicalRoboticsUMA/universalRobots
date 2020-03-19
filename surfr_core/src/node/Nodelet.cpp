/*
 *
 *  surfr_core/node/Nodelet.cpp
 * 
 *               Generic nodelet class to load and link data with ROS
 *               -----------------------------------------------------------
 *  Begin Date : October 18, 2017
 *  Revision   : October 25, 2017 (rev 3)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               -----------------------------------------------------------
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

#include "surfr_core/node/Nodelet.h"


PLUGINLIB_EXPORT_CLASS(surfr::Nodelet, nodelet::Nodelet)


namespace surfr {

Nodelet::Nodelet() :
  _base(NULL) {
  NODELET_DEBUG("New nodelet object created.");
}

Nodelet::~Nodelet() {
  if (_base) delete _base;
  NODELET_DEBUG("Nodelet object deleted.");
}

void Nodelet::onInit() {
 /**
  * The Surgical Framework can also receive arguments from the command line.
  * It does not depend on ROS, so you must only take care of call it before using
  * the Surgical Framework.
  */
  surfr::init(this->getMyArgv().size(), this->getMyArgv());
  
 /**
  * The main() function receives some standard arguments from ROS in addition to those
  * passed by command line. More specifically, the "__name" argument gives the name
  * of the node given at time of execution. The purpose of this parameter is twohold:
  * on one hand it allows the selection of the configuration parameters for a specific
  * device, and on the other hand it is used for loading the appropiate driver.
  * 
  * This section does not need to check wether the argument is found, as ROS is
  * supposed to always pass the node name as argument.
  */
  std::string classname, nodename, model;
  uint32_t    id;
  nodename = (this->getName()).substr(1);
  model    = nodename.substr(0, nodename.rfind('_'));
  id       = atoi(nodename.substr(nodename.rfind('_') + 1).c_str());
  for (std::vector<std::string>::const_iterator it  = this->getMyArgv().begin();
                                                it != this->getMyArgv().end();
                                                it++) {
    if ((*it).find("__class") != std::string::npos) {
      classname = *it;
      classname.erase(0, classname.find('=') + 1);
      ROS_DEBUG_STREAM("Class Name: " << classname);
      break;
    }
  }

  ROS_DEBUG_STREAM("Node name : " << nodename);
  ROS_DEBUG_STREAM("Model     : " << model);
  ROS_DEBUG_STREAM("Id        : " << id);
  //! MAIN RESULTS: classname, nodename, model, id
  
 /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  _nh = &this->getPrivateNodeHandle();
  //! MAIN RESULT: nh

 /**
  * Read the ROS parameters of the NodeHandle related to the selected "model" device.
  */
  XmlRpc::XmlRpcValue yaml;
  _nh->getParam("", yaml);
  surfr::Params setup;
  surfr::parse_yaml(setup, yaml, "", model);
  //! MAIN RESULT: setup

 /**
  * Initialization of the Surgical Framework Device
  */
  _base = new pluginlib::ClassLoader<surfr::Base>("surfr_core", "surfr::Base");
  try {
    _dev = _base->createInstance("surfr::dev::" + classname);
    _dev->init(setup, id);
    _dev->configure();
  } catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  if (_dev->start()) {
    NODELET_DEBUG("Initializing nodelet...");
    _node.reset(new Node(this->getPrivateNodeHandle(), _dev));
    _thread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&Nodelet::controller_poll, this)));
  } else {
    NODELET_ERROR("There was a problem when creating the surfr::dev::DeviceURX object.");
    exit(1);
  }
}

void Nodelet::controller_poll() {
  NODELET_DEBUG("Starting nodelet thread...");

  while (ros::ok()) {
    _node->update();

    ros::spinOnce(); 
  }
}

}
