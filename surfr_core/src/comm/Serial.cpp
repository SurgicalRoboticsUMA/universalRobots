/*
 *
 *  surfr_core/comm/Serial.cpp
 * 
 *               Serial port communication class
 *               ----------------------------------------------------------
 *  Begin Date : September 11, 2018
 *  Revision   : January 29, 2019 (rev 1)
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

#include "surfr_core/comm/Serial.h"

PLUGINLIB_EXPORT_CLASS(surfr::comm::Serial, surfr::Base)


namespace surfr { namespace comm {

Serial::Serial() :
  _handle(NULL),
  _port(""),
  _baudrate(0),
  _isResponse(false),
  //~ _packetCount(0),
  _lastRequest("") {
  SURFR_DEBUG("Done");
}

void Serial::init(surfr::Params& cfg, unsigned int id) {
  this->Base::init(cfg, id);

  if (!this->is_loaded()) {
    SURFR_ERROR("Serial Port base class not correctly initialized, constructor aborted.");
    return;
  }

  SURFR_DEBUG("Done");
}

Serial::~Serial() {
/*
 * Destruction of the communication handles and buffer
 */
  if (this->is_running()) this->stop();
  if (_buffer.size()) {
    for (std::map<std::string, uint8_t*>::iterator it = _buffer.begin(); it != _buffer.end(); it++) {
      delete it->second;
    }
  }

  SURFR_DEBUG("Serial Port " + this->port() + " destroyed.");
}

bool Serial::configureHook() {
  bool                ret = true;
  const surfr::Params cfg = _config;

/*
 * Extraction of the configuration parameters
 */
  // SERIAL PORT
  if ( ret &= (cfg.find("port") != cfg.end()) ) {
    _port = std::string((XmlRpc::XmlRpcValue) cfg.at("port"));
    SURFR_DEBUG("Port: " + _port);
  } else {
    SURFR_ERROR("Undefined Port address");
  }
  // SERIAL BAUD RATE
  if ( ret &= (cfg.find("baudrate") != cfg.end()) ) {
    _baudrate = int((XmlRpc::XmlRpcValue) cfg.at("baudrate"));
    SURFR_DEBUG("Baud rate: " + std::to_string(_baudrate));
  } else {
    SURFR_ERROR("Undefined Port address");
  }
  // LIST OF COMMANDS AVAILABLE TO USE WITH THE SOCKET
  if ( ret &= (cfg.find("commands") != cfg.end()) ) {
    XmlRpc::XmlRpcValue tmp_cmds = cfg.at("commands");
    parse_yaml(_commands, tmp_cmds, "", "");
  } else {
    SURFR_ERROR("Commands list not found (socket is useless without a command list).");
  }

  if (ret) {
    SURFR_DEBUG("Serial Port " + this->port() + " configured.");
  }
  return ret;
}

bool Serial::startHook() {
  bool ret = true;

 /*
  * Opening the communication socket with the device 
  */
  // Set initial parameters of the Serial Port
  try {
    _handle = new boost::asio::serial_port(_ioService, this->port());
    _handle->set_option( boost::asio::serial_port_base::character_size( 8 ) );
    this->baud(_baudrate);
  } catch (std::exception &e) {
    SURFR_ERROR(e.what() << ". Maybe you don't have access permission (use 'sudo usermod -a -G dialout $USER' and reboot).");
    ret = false;
  }

  if (ret) {
    //~ _packetCount = 0;
    _state       = STATE_RUNNING;
    SURFR_INFO("Serial Port " + this->port() + " started.");
  }
  return ret;
}

bool Serial::stopHook() {
  if (_handle) delete _handle;

  SURFR_INFO("Serial Port " + this->port() + " stopped.");
  return true;
}

uint32_t Serial::baud() {
  return _baudrate;
}
void Serial::baud(const uint32_t& rate) {
  _baudrate = rate;
  _handle->set_option( boost::asio::serial_port_base::baud_rate( _baudrate ) );
}

std::string Serial::port() {
  return _port;
}

std::vector<std::string> Serial::commands() {
  std::vector<std::string> ret;

  for (Params::iterator it = _commands.begin(); it != _commands.end(); it++) {
    std::string kCmd = ((it->first).find('/') != std::string::npos) ? (it->first).substr(0, (it->first).find('/')) : (it->first);

    if (std::find(ret.begin(), ret.end(), kCmd) == ret.end()) {
      ret.push_back(kCmd);
    }
  }

  return ret;
}

bool Serial::send(std::string cmd) {
  try {
    boost::asio::write(*_handle, boost::asio::buffer(cmd.c_str(), cmd.size()));
  } catch(std::exception &e) {
    std::cerr << "Could not send command '" << cmd << "' towards port " << _port << "." << std::endl;
    return false;
  }
  
  return true;
}

bool Serial::recv(std::string& data) {
  try {
    boost::asio::streambuf response;

    boost::asio::read_until(*_handle, response, '\r'); 
    boost::asio::streambuf::const_buffers_type buf = response.data();

    data = std::string(boost::asio::buffers_begin(buf), boost::asio::buffers_begin(buf) + response.size());
  } catch(std::exception &e) {
    //~ std::cerr << "There was an error while receiving command '" << cmd << "' towards port " << _port << "." << std::endl;
    SURFR_ERROR(e.what());
    return false;
  }

  return true;
}


void Serial::input(Flow& data) {
  std::vector<uint8_t> packet;
  std::string          command = std::string(FLOW_CAST(data, XmlRpc::XmlRpcValue, "command"));
  XmlRpc::XmlRpcValue  params  = surfr::unparse_params(_commands, command);

  if (params.hasMember("request")) {
    for (uint16_t k = 0; k < params["request"].size(); k++) {
      XmlRpc::XmlRpcValue kValue;
      std::string         kParam;

      if (params["request"][k].getType() == XmlRpc::XmlRpcValue::TypeString) {
        kParam = std::string(params["request"][k]);
        
        if (params.hasMember(kParam)) {
          kValue = (data.find(kParam) != data.end()) ? FLOW_CAST(data, XmlRpc::XmlRpcValue, params["request"][k]) : params[kParam];
// IMPORTANT NOTE: ACTUALLY, THE FORMAT OF THIS INPUT VALUE IS RESPONSABILITY OF THE USER.
//                 IN THE FUTURE, THIS CODE SHOULD CHECK THE FORMAT PROVIDED BY THE YAML
//                 FILE WITH THE INPUT DATA
        } else {
          kValue = params["request"][k];
        }
      } else {
        kValue = params["request"][k];
      }

    // 4. Add next parameter to requested packet
      if (kValue.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        // This considers overflow values and splits into some bytes
        uint32_t kInt = abs(int(kValue));
        while (kInt > 255) {
          kInt = kInt % 256;
          packet.push_back(kInt);
        }
        packet.push_back(kInt);
      } else if (kValue.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (uint32_t i = 0; i < kValue.size(); i++) {
          uint32_t kInt = abs(int(kValue[i]));
          while (kInt > 255) {
            kInt = kInt % 256;
            packet.push_back(kInt);
          }
          packet.push_back(kInt);
        }
      } else if (kValue.getType() == XmlRpc::XmlRpcValue::TypeString) {
          for (uint32_t i = 0; i < kValue.size(); i++) {
            packet.push_back(std::string(kValue)[i]);
// std::cout << std::string(kValue)[i];
          }
      } 
    }
// std::cout << std::endl;
    // _lastRequest = command;
    packet.push_back('\r');
//~ std::cout << "Request:" << std::string(packet.begin(), packet.end()) << std::endl;
    this->send(std::string(packet.begin(), packet.end()));
  }
  _lastRequest = command;

    // This is just for debug purposes
    // std::stringstream ss_packet;
    // ss_packet << "Requested packet (" << std::dec << int(packet.size()) << " bytes) for command '" << _lastRequest << "': 0x[";
    // for (uint32_t k = 0; k < packet.size() - 1; k++) {
    //   ss_packet << std::setfill('0') << std::setw(2*sizeof(uint8_t)) << std::hex << int(packet[k]) << ",";
    // }
    // ss_packet << std::setfill('0') << std::setw(2*sizeof(uint8_t)) << std::hex << int(packet.back()) << "]";
    // SURFR_DEBUG(ss_packet.str());
_isResponse = true;
  data.clear();
}

void Serial::output(Flow& data) {
  if (_isResponse) {
  //~ uint8_t* kPtr = _buffer[_lastRequest];
// NOTE: Actually, response only admits 1 parameter.
  std::string response;
    this->recv(response);
  //~ std::string::iterator kPtr = response.begin();
  
  for (uint16_t kList = 0; kList < _commands[_lastRequest + "/response"].size(); kList++) {
    std::string         kParam = _commands[_lastRequest + "/response"][kList];
    //~ std::string         kSeparator = (kList < _commands[_lastRequest + "/response"].size()-1) ? _commands[_lastRequest + "/response"][kList+1] : "";

    if (_commands.find(_lastRequest + "/" + kParam) != _commands.end()) {
      XmlRpc::XmlRpcValue kValue  = _commands[_lastRequest + "/" + kParam];
      std::string         kType   = std::string(kValue[0]);
      uint16_t            kSizeof = int(kValue[1]);
      uint32_t            kDim    = (kType.rfind("[") == std::string::npos) ? 1 :
                                    std::stoi(kType.substr(kType.rfind("[")+1, kType.size() - kType.rfind("]")));

      if (kType.find("Unsigned") != std::string::npos) {
        (_out)[kParam] = std::stoul(response);
      } else if (kType.find("Integer") != std::string::npos) {
        (_out)[kParam] = std::stoi(response);
      } else if (kType.find("Double") != std::string::npos) {
        (_out)[kParam] = std::stod(response);
      } else if (kType.find("String") != std::string::npos) {
        (_out)[kParam] = response;
      }
//~ std::cout << "Response [" << kParam << "]: " << FLOW_CAST(_out, std::string, kParam) << std::endl;
    } else {
      SURFR_ERROR("Parameter " + kParam + " not found in response list.");
    }
  }
  }

  data.clear();
  data = _out;
  _isResponse = false;
}
bool Serial::service(Flow& data) {
  // Service message
  std::string srvName = FLOW_CAST(data, std::string, "service");
  std::string srvType = FLOW_CAST(data, std::string, "type");
  std::string srvData = std::string(FLOW_CAST(data, XmlRpc::XmlRpcValue, "data"));

  if (boost::iequals("string", srvType)) {
    SURFR_DEBUG("Service " << srvName << " sending packet: " + srvData);  
    // this->send(srvData.c_str());
  } else {
    SURFR_WARNING("Service " << srvName << " did not receive a string type");
  }

  return true;
}

} // namespace comm
} // namespace surfr
