/*
 *
 *  surfr_core/node/main_device.cpp
 * 
 *               Generic node creation to load and link a device with ROS
 *               ----------------------------------------------------------
 *  Begin Date : September 12, 2016
 *  Revision   : January 24, 2019 (rev 11)
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

#include <surfr_core/node/Node.h>
//~ #include <surfr_core/datatypes.h>


int main(int argc, char **argv)
{
// ***** BEGIN TESTEO DATATYPES.H BORRABLE!!! *****
  // const char cc[] = "Hi const!";
  // std::string cs = "Hi! 2";
  // const XmlRpc::XmlRpcValue cx(int(3));
  // surfr::Scalar sb(bool(true)), su(uint64_t(5)), si(int64_t(-4)), sd(double_t(3.14)), ss("2.78"), ssb(cs);
  // surfr::Scalar sb2 = bool(true), su2 = uint64_t(5), si2 = int64_t(-4), sd2 = double_t(3.14), ss2 = std::string("Hi cast!");
  // surfr::Scalar sb3 = true, su3 = 5, si3 = -4, sd3 = 3.14, ss3 = "Hi!", ss4 = cc, ss5 = cs;
  // surfr::Scalar sx(cx), sx2 = cx;
  // std::cout << "Boolean ("   << sb.type_desc() << "): " << bool(sb) << " / " << unsigned(sb) << " / " << int(sb) << " / " << double(sb) << " / " << std::string(sb) << std::endl;
  // std::cout << "Unsigned ("  << su.type_desc() << "): " << bool(su) << " / " << unsigned(su) << " / " << int(su) << " / " << double(su) << " / " << std::string(su) << std::endl;
  // std::cout << "Integer ("   << si.type_desc() << "): " << bool(si) << " / " << unsigned(si) << " / " << int(si) << " / " << double(si) << " / " << std::string(si) << std::endl;
  // std::cout << "Double ("    << sd.type_desc() << "): " << bool(sd) << " / " << unsigned(sd) << " / " << int(sd) << " / " << double(sd) << " / " << std::string(sd) << std::endl;
  // std::cout << "String ("    << ss.type_desc() << "): " << bool(ss) << " / " << unsigned(ss) << " / " << int(ss) << " / " << double(ss) << " / " << std::string(ss) << std::endl;
  // std::cout << "Xml Value (" << sx.type_desc() << "): " << bool(sx) << " / " << unsigned(sx) << " / " << int(sx) << " / " << double(sx) << " / " << std::string(sx) << std::endl;
  // std::cout << "sb: " << sb << " (" << sb.type_desc() << ")" << std::endl;
  // std::cout << "su: " << su << " (" << su.type_desc() << ")" << std::endl;
  // std::cout << "si: " << si << " (" << si.type_desc() << ")" << std::endl;
  // std::cout << "sd: " << sd << " (" << sd.type_desc() << ")" << std::endl;
  // std::cout << "ss: " << ss << " (" << ss.type_desc() << ")" << std::endl;
  // std::cout << "sx: " << sx << " (" << sx.type_desc() << ")" << std::endl;

  // surfr::Scalar addb = sb + "5",
  //               addu = 5 + su,
  //               addi = si + std::to_string(5),
  //               addd = 5 + sd,
  //               adds = ss2 + 5;
  // std::cout << "suma sb: " << addb << " (" << addb.type_desc() << ")" << std::endl;
  // std::cout << "suma su: " << addu << " (" << addu.type_desc() << ")" << std::endl;
  // std::cout << "suma si: " << addi << " (" << addi.type_desc() << ")" << std::endl;
  // std::cout << "suma sd: " << addd << " (" << addd.type_desc() << ")" << std::endl;
  // std::cout << "suma ss: " << adds << " (" << adds.type_desc() << ")" << std::endl;
  // // exit(0);
// ***** END TESTEO DATATYPES.H BORRABLE!!! *****

 /**
  * The Surgical Framework can also receive arguments from the command line.
  * It does not depend on ROS, so you must only take care of call it before using
  * the Surgical Framework.
  */
  surfr::init(argc, argv);

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
  uint32_t    id;
  std::string classname, nodename, model;
XmlRpc::XmlRpcValue args;

  for (uint16_t it = 0; it < argc; it++) {
    std::string it_str;
      it_str = argv[it];
    ROS_DEBUG_STREAM("Param " << it << ": " << it_str);
    if (it_str.find("__name") != std::string::npos) {
      nodename = it_str;
        nodename.erase(0, nodename.find('=') + 1);
      model    = nodename.substr(0, nodename.rfind('_'));
      id       = atoi(nodename.substr(nodename.rfind('_') + 1).c_str());

      ROS_DEBUG_STREAM("Node name : " << nodename);
      ROS_DEBUG_STREAM("Model     : " << model);
      ROS_DEBUG_STREAM("Id        : " << id);
args["__name"] = nodename;
      continue;
    }
    if (it_str.find("__class") != std::string::npos) {
      classname = it_str;
        classname.erase(0, classname.find('=') + 1);
      ROS_DEBUG_STREAM("Class Name: " << classname);
args["__class"] = classname;
      continue;
    }
  }
  //! MAIN RESULTS: classname, nodename, model, id

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
  * Initialization of the Surgical Framework Device
  */
  boost::shared_ptr<surfr::Base>      device;
  pluginlib::ClassLoader<surfr::Base> base("surfr_core", "surfr::Base");
  try {
    device = base.createInstance("surfr::dev::" + classname);
    device->init(setup, id);
    device->configure();
  } catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("%s: %s", classname.c_str(), ex.what());
    exit(1);
  }
  //! MAIN RESULT: device

  if (device->start()) {
    surfr::Node handle(nh, device);
 /**
  * There are two methods to check for various states of shutdown. The most common is ros::ok().
  * When Ctrl-C is pressed, the node is shutdown by the master. Once ros::ok() returns false, the
  * node has finished shutting down.
  */
    while (ros::ok()) {
 /**
  * surfr::DeviceNode::update() updates all output topics and tf transforms related to
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
  }

 /**
  * Before ending the node, it is recommended to delete all params related to this node.
  * This way, no configuration data is kept into ROS execution and node ends cleanly.
  */
  nh.deleteParam("");

  return 0;
}
