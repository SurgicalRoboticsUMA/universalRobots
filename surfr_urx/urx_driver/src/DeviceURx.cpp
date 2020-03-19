/*
 *
 *  urx_driver/DeviceURx.cpp
 * 
 *               Universal Robots Device class
 *               ----------------------------------------------------------
 *  Begin Date : September 2, 2016
 *  Revision   : September 11, 2018 (rev 21)
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


#include "urx_driver/DeviceURx.h"

PLUGINLIB_EXPORT_CLASS(surfr::dev::DeviceURx, surfr::Base)


namespace surfr { namespace dev {

DeviceURx::DeviceURx() :
  _socketIn(NULL),
  _socketOut(NULL) {
  SURFR_DEBUG("Done");
}

DeviceURx::~DeviceURx() {
  /*
   * Destruction of the communication handles
   */
    if (this->is_running()) this->stop();
    if (_socketIn)          delete _socketIn;
    if (_socketOut)         delete _socketOut;
  
    SURFR_DEBUG("Done");
}
  
bool DeviceURx::configureHook() {
  bool ret = true;

  /*
   * Creation of the communication parameters
   */
  ret &= (_config.find("sockets/version") != _config.end());
  if (!_is_simulation && ret) {
    // Validation of the socket parameters
    ret &= (_config.find("sockets/ip") != _config.end()) &&
           //~ (_config.find("sockets/packet_size_" + std::string(_config.at("sockets/version"))) != _config.end()) &&
           (_config.find("sockets/port_in") != _config.end()) &&
           (_config.find("sockets/port_out") != _config.end());
    if (ret) {
      SURFR_INFO("Configuring real mode...");

      // Creation of the communication handles
      Params cfgSocket;
        cfgSocket["simulation"]                  = XmlRpc::XmlRpcValue(false);
        cfgSocket["frequency"]                   = _config.at("frequency");
        cfgSocket["models/Ethernet/description"] = XmlRpc::XmlRpcValue("Ethernet");
        //~ cfgSocket["packet_size"]                 = _config.at("sockets/packet_size_" + std::string(_config.at("sockets/version")));
        cfgSocket["ip"]                          = _config.at("sockets/ip");
        cfgSocket["protocol"]                    = XmlRpc::XmlRpcValue("tcp");
        // cfgSocket["port"] = _config.at("sockets/port_out");
        // _socketOut = new surfr::comm::Ethernet();

        std::string path_port = "sockets/" + std::string(cfgSocket.at("protocol")) + "_" + std::string(_config.at("sockets/port_out"));
        cfgSocket["port"]     = std::stoi(std::string(_config.at("sockets/port_out")));
        cfgSocket["commands"] = surfr::unparse_params(_config, path_port);
        _socketOut = new surfr::comm::Ethernet();
        _socketOut->init(cfgSocket, 2);
        ret &= _socketOut->configure();

        path_port = "sockets/" + std::string(cfgSocket.at("protocol")) + "_" + std::string(_config.at("sockets/port_in"));
        cfgSocket["port"] = std::stoi(std::string(_config.at("sockets/port_in")));
        cfgSocket["commands"] = surfr::unparse_params(_config, path_port);
        _socketIn  = new surfr::comm::Ethernet();
        _socketIn->init(cfgSocket, 1);
        ret &= _socketIn->configure();


      SURFR_DEBUG("Socket Parameters:");
      SURFR_DEBUG("  Description  : " + std::string(cfgSocket.at("models/Ethernet/description")));
      //~ SURFR_DEBUG("  Packet size  : " + std::to_string((int) cfgSocket.at("packet_size")));
      SURFR_DEBUG("  Frequency    : " + std::to_string((int) cfgSocket.at("frequency")));
      SURFR_DEBUG("  Input (mode) : " + std::string(cfgSocket["ip"]) + ":"
                                      + std::string(_config["sockets/port_in"])  + " (output)");
      SURFR_DEBUG("  Output (mode): " + std::string(cfgSocket["ip"]) + ":"
                                      + std::string(_config["sockets/port_out"]) + " (input)");
    }
  }

  if (ret) {
    SURFR_INFO("DeviceURx '" + this->name() + "' configured.");
  } else {
    SURFR_WARNING("DeviceURx '" + this->name() + "' could not be configured.");
  }
  return ret;
}

bool DeviceURx::startHook() {
  bool ret = true;
/*
 * Start of the communication parameters
 */
  if (!_is_simulation) {
    if (!(ret &= _socketIn->start())) {
      SURFR_WARNING("Input port not connected.");
      // _state = surfr::STATE_EXCEPTION;
    } else if (!(ret &= _socketOut->start())) {
      SURFR_WARNING("Output port not connected.");
      // _state = surfr::STATE_EXCEPTION;
    }
  }

  if (ret) {
    SURFR_INFO("DeviceURx '" + this->name() + "' started.");
    Flow cmd;
    cmd["command"] = XmlRpc::XmlRpcValue("reading_" + std::string(_config.at("sockets/version")));
    _socketOut->input(cmd);
  } else {
    if (_socketIn)  delete _socketIn;
    if (_socketOut) delete _socketOut;
    ret            = true;
    _socketIn      = NULL;
    _socketOut     = NULL;
    _is_simulation = true;
    _config.at("simulation") = _is_simulation;
    SURFR_INFO("DeviceURx '" + this->name() + "' started in simulation mode.");
  }
  return ret;
}

bool DeviceURx::stopHook() {
  bool ret = true;
 /*
  * Close all opened socket handles
  */
  if (!_is_simulation) {
    ret &= _socketIn->stop();
    ret &= _socketOut->stop();
  }

  if (ret) {
    SURFR_INFO("DeviceURx '" + this->name() + "' stopped.");
  } else {
    SURFR_WARNING("DeviceURx '" + this->name() + "' not stopped.");
  }
  return ret;
}

void DeviceURx::output(Flow& data) {
  // 1. Read from your Device Library
  Flow read;
  _socketOut->output(read);
  // 2. Update all gathered data into _out attribute map
  (_out)["joints/state"]       = read["state"];
  (_out)["joints/position"]    = read["q_actual"];
  (_out)["joints/velocity"]    = read["qd_actual"];
  (_out)["joints/current"]     = read["i_actual"];
  (_out)["joints/temperature"] = read["t_actual"];
  (_out)["pose"]               = read["pose_actual"];
  (_out)["velocity"]           = read["speed_actual"];
  (_out)["acceleration"]       = read["accel_actual"];
  (_out)["force"]              = read["force_actual"];
  // 3. Update output "data" value passed by reference 
  data = _out;
}

void DeviceURx::input(Flow& data) {
  std::string command = FLOW_CAST(data, std::string, "topic"),
              type    = FLOW_CAST(data, std::string, "type");
  bool                  isMove   = command.find("move")   != std::string::npos,
                        isSpeed  = command.find("speed")  != std::string::npos,
                        isPose   = command.find("pose")   != std::string::npos,
                        isJoints = command.find("joints") != std::string::npos;
  std::string           datatype = "data";
  std::vector<double_t> value;
  Flow                  packet;

  // Datatype initialization
  if (type == "sensor_msgs/JointState") {
    if (isMove)  datatype = "data/position";
    if (isSpeed) datatype = "data/velocity";
  }
  // Value initialization
  try {
    XmlRpc::XmlRpcValue tmpValue = FLOW_CAST(data, XmlRpc::XmlRpcValue, datatype);
    for (int k = 0; k < tmpValue.size(); k++) {
      value.push_back(double(tmpValue[k]));
    }
  } catch (boost::exception const& err) {
    value = FLOW_CAST(data, std::vector<double_t>, datatype);
  }
  // Packet map generation
  if (isJoints) {
    packet["q1"] = XmlRpc::XmlRpcValue(std::to_string(value[0]));
    packet["q2"] = XmlRpc::XmlRpcValue(std::to_string(value[1]));
    packet["q3"] = XmlRpc::XmlRpcValue(std::to_string(value[2]));
    packet["q4"] = XmlRpc::XmlRpcValue(std::to_string(value[3]));
    packet["q5"] = XmlRpc::XmlRpcValue(std::to_string(value[4]));
    packet["q6"] = XmlRpc::XmlRpcValue(std::to_string(value[5]));
  } else if (isPose) {
    packet["px"] = XmlRpc::XmlRpcValue(std::to_string(value[0]));
    packet["py"] = XmlRpc::XmlRpcValue(std::to_string(value[1]));
    packet["pz"] = XmlRpc::XmlRpcValue(std::to_string(value[2]));
    packet["rx"] = XmlRpc::XmlRpcValue(std::to_string(value[3]));
    packet["ry"] = XmlRpc::XmlRpcValue(std::to_string(value[4]));
    packet["rz"] = XmlRpc::XmlRpcValue(std::to_string(value[5]));
  }
  packet["command"] = XmlRpc::XmlRpcValue(command);

  SURFR_DEBUG("Command: " + command);
  SURFR_DEBUG("Type   : " + type);
  SURFR_DEBUG("Time   : " + std::to_string(FLOW_CAST(data, double  , "time" )));
  SURFR_DEBUG("Data   : " << value);
  _socketIn->input(packet);
}

bool DeviceURx::service(Flow& data) {
  this->input(data);

  return true;
}

} // namespace dev
} // namespace surfr
