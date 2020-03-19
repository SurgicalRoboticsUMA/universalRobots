/*
 *
 *  surfr_core/main_socket.cpp
 * 
 *               Generic communication node to load and link data with ROS
 *               ----------------------------------------------------------
 *  Begin Date : November 14, 2016
 *  Revision   : July 17, 2017 (rev 3)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  Done List  : - Fixed an initialization problem (R3)
 *               - Using global YAML parsing function from core.h (R2)
 *               - Initial release (R1)
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
#include "surfr_core/comm/Ethernet.h"



int main(int argc, char **argv)
{
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
  int         id;
  std::string nodename, model;
  
  for (int it = 0; it < argc; it++) {
    nodename = argv[it];
    if (nodename.find("__name") != std::string::npos) {
      nodename.erase(0, nodename.find('=') + 1);
      model = nodename.substr(0, nodename.rfind('_'));
      id = atoi(nodename.substr(nodename.rfind('_') + 1).c_str());
      break;
    }
  }
std::cout << "Node name: " << nodename << " (id = " << id << ")" << std::endl;
  //! MAIN RESULTS: nodename, model, id
  
 /**
  * The Surgical Framework can also receive arguments from the command line.
  * It does not depend on ROS, so you must only take care of call it before using
  * the Surgical Framework.
  */
  surfr::init(argc, argv);
 /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, nodename);
 /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh("~");
  //! MAIN RESULT: nh

 /**
  * Read the ROS parameters of the NodeHandle related to the selected "model" device.
  */
  XmlRpc::XmlRpcValue yaml;
  nh.getParam("", yaml);
  surfr::Params setup;
  surfr::parse_yaml(setup, yaml, "", model);
  //! MAIN RESULT: setup

 /**
  * Ethernet socket object creation and initialization. If socket fails on start
  * then the program is stopped.
  */
  // surfr::comm::Ethernet socket(setup, id);
  // ROS_ASSERT(socket.start());
  pluginlib::ClassLoader<surfr::Base> base("surfr_core", "surfr::Base");
  boost::shared_ptr<surfr::Base> socket;
  try {
    socket = base.createInstance("surfr::comm::Ethernet");
    socket->init(setup, id);
    socket->configure();
    ROS_ASSERT(socket->start());
  } catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("%s", ex.what());
  }
  //! MAIN RESULT: socket

 /**
  * Ethernet socket node handle for ROS
  */
  surfr::Node handle(nh, socket);
  //! MAIN RESULT: handle

 /**
  * There are two methods to check for various states of shutdown. The most common is ros::ok().
  * When Ctrl-C is pressed, the node is shutdown by the master. Once ros::ok() returns false, the
  * node has finished shutting down.
  */
  while (ros::ok()) {
 /**
  * surfr::Node.update() updates all output topics and tf transforms related to
  * this device.
  */
    handle.update();

 /**
  * ros::spinOnce() calls all the callbacks waiting to be called at that point in time.
  * spinOnce() is meant for single-threaded applications, and is not optimized for being
  * called from multiple threads at once.
  */
    ros::spinOnce();
  }

  return 0;
}
