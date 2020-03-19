/*
 *
 *  surfr_core/comm/Ethernet.cpp
 * 
 *               Ethernet Communication class
 *               ----------------------------------------------------------
 *  Begin Date : November 13, 2016
 *  Revision   : October 31, 2018 (rev 15)
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

#include "surfr_core/comm/Ethernet.h"

PLUGINLIB_EXPORT_CLASS(surfr::comm::Ethernet, surfr::Base)


namespace surfr { namespace comm {

Ethernet::Ethernet() :
  _protocol(PROTOCOL::NONE2),
  _ip(""),
  _port(0),
  _packetCount(0),
  _lastRequest(""),
  _udpSocket(NULL),
  _tcpSocket(NULL) {
  SURFR_DEBUG("Done");
}

void Ethernet::init(surfr::Params& cfg, unsigned int id) {
  this->Base::init(cfg, id);

  if (!this->is_loaded()) {
    SURFR_ERROR("Ethernet socket base class not correctly initialized, constructor aborted.");
    return;
  }

  SURFR_DEBUG("Done");
}

Ethernet::~Ethernet() {
/*
 * Destruction of the communication handles and buffer
 */
  if (this->is_running()) this->stop();
  if (_tcpResolver)       delete _tcpResolver;
  if (_tcpSocket)         delete _tcpSocket;
  if (_buffer.size()) {
    for (std::map<std::string, uint8_t*>::iterator it = _buffer.begin(); it != _buffer.end(); it++) {
      delete it->second;
    }
  }

  SURFR_DEBUG("Socket " + this->address() + " destroyed.");
}

bool Ethernet::configureHook() {
  bool                ret = true;
  const surfr::Params cfg = _config;

/*
 * Extraction of the configuration parameters
 */
  // SOCKET IP ADDRESS
  if ( ret &= (cfg.find("ip") != cfg.end()) ) {
    _ip = std::string((XmlRpc::XmlRpcValue) cfg.at("ip"));
    SURFR_DEBUG("IP: " + _ip);
  } else {
    SURFR_ERROR("Undefined IP address");
  }
  // SOCKET PORT
  if ( ret &= (cfg.find("port") != cfg.end()) ) {
    _port = int((XmlRpc::XmlRpcValue) cfg.at("port"));
    SURFR_DEBUG("Port: " + std::to_string(_port));
  } else {
    SURFR_ERROR("Undefined Port address");
  }
  // SOCKET COMMUNICATION MODE: UDP/TCP
  std::string tmpProtocol = "UDP";
  if (cfg.find("protocol") != cfg.end()) {
    tmpProtocol = std::string((XmlRpc::XmlRpcValue) cfg.at("protocol"));
    boost::to_upper(tmpProtocol);
  }
  if        (!tmpProtocol.compare("UDP")) {
    _protocol    = PROTOCOL::UDP;
    _udpSocket   = new boost::asio::ip::udp::socket(_ioService);
  } else if (!tmpProtocol.compare("TCP")) {
    _protocol    = PROTOCOL::TCP;
    _tcpResolver = new boost::asio::ip::tcp::resolver(_ioService);
    _tcpSocket   = new boost::asio::ip::tcp::socket(_ioService);
  } else {
    _protocol = PROTOCOL::NONE2;
  }
  if ( ret &= (_protocol != PROTOCOL::NONE2) ) {
    SURFR_DEBUG("Communication protocol: " + PROTOCOL_DESC.at(_protocol));
  } else {
    SURFR_ERROR("Communication protocol " + tmpProtocol + " is not valid. Please set to UDP or TCP.");
  }
  // LIST OF COMMANDS AVAILABLE TO USE WITH THE SOCKET
  if ( ret &= (cfg.find("commands") != cfg.end()) ) {
    XmlRpc::XmlRpcValue tmp_cmds = cfg.at("commands");
    parse_yaml(_commands, tmp_cmds, "", "");
  } else {
    SURFR_ERROR("Commands list not found (socket is useless without a command list).");
  }
  // Initialization of response buffers and packet sizes
  std::vector<std::string> cmd = this->commands();
  for (std::vector<std::string>::iterator it = cmd.begin(); it != cmd.end(); it++) {
    this->packet_size(*it);
  }

  if (ret) {
    SURFR_DEBUG("Socket " + this->address() + " configured.");
  }
  return ret;
}

bool Ethernet::startHook() {
  bool ret = true;

 /*
  * Opening the communication socket with the device 
  */
  switch (_protocol) {
  // Construct UDP socket
  case PROTOCOL::UDP: {
    boost::asio::ip::udp::endpoint remote_endpoint( boost::asio::ip::address_v4::from_string(_ip), _port);
    _udpSocket->open(boost::asio::ip::udp::v4());
    _udpSocket->connect(remote_endpoint);
    // This thread keeps reading response data from endpoint
    _thread = boost::thread(&Ethernet::_cbUdpRead, this);
    break; }
  // Construct TCP socket
  case PROTOCOL::TCP: {
    boost::asio::ip::tcp::resolver::iterator remote_endpoint;
      remote_endpoint = _tcpResolver->resolve(boost::asio::ip::tcp::resolver::query(_ip,
                                                                                    std::to_string(_port)));
#if ((BOOST_VERSION / 1000000 == 1) && ((BOOST_VERSION / 100) % 1000 >= 48))
    boost::asio::connect(*_tcpSocket, remote_endpoint);
#else
	  _tcpSocket->connect(*remote_endpoint);
#endif
    boost::asio::async_read(*_tcpSocket,
                            boost::asio::buffer(_buffer[_lastRequest], this->packet_size() + ((BOOST_VERSION / 1000000 == 1) && ((BOOST_VERSION / 100) % 1000 >= 48))),
                            boost::bind(&Ethernet::_cbTcpRead,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
    // This thread keeps the I/O service running in parallel
    _thread = boost::thread(&Ethernet::_cbIoService, this);

    //~ boost::asio::ip::tcp::resolver::iterator remote_endpoint = _tcpResolver->resolve(boost::asio::ip::tcp::resolver::query(_ip, std::to_string(_port)));
    //~ #if ((BOOST_VERSION / 1000000 == 1) && ((BOOST_VERSION / 100) % 1000 >= 48))
      //~ boost::asio::connect(*_tcpSocket, remote_endpoint);
    //~ #else
	  //~ _tcpSocket->connect(*remote_endpoint);
    //~ #endif
    // This thread keeps reading response data from endpoint
    // _thread = boost::thread(&Ethernet2::_cbTcpRead, this);
    break; }
  default: {
    SURFR_DEBUG("Socket protocol not defined, nothing done");
    ret = false; }
  }
  
  if (ret) {
    _packetCount = 0;
    _state       = STATE_RUNNING;
    SURFR_INFO("Socket " + this->address() + " started with " + PROTOCOL_DESC.at(_protocol) + " protocol.");
  }
  return ret;
}

bool Ethernet::stopHook()
{
 /*
  * Close all opened socket handles
  */
  if (_udpSocket) _udpSocket->close();
  if (_tcpSocket) _tcpSocket->close();

  SURFR_INFO("Socket " + this->address() + " stopped.");
  return true;
}

std::string Ethernet::protocol() {
  std::string lower = PROTOCOL_DESC.at(_protocol);
    boost::to_lower(lower);
  return lower;
}
void Ethernet::protocol(PROTOCOL protocol) {
  if (this->is_loaded()) {
    _protocol = protocol;
    SURFR_DEBUG(std::string("Socket protocol set to PROTOCOL::") + PROTOCOL_DESC.at(_protocol));
  } else {
    SURFR_DEBUG(std::string("Socket protocol can only be set at State::") + this->state_desc(surfr::StateMachine::STATE_LOADED));
  }
}

std::string Ethernet::ip() {
  return _ip;
}
uint16_t Ethernet::port() {
  return _port;
}
std::string Ethernet::address() {
  return (_ip + ":" + std::to_string(_port));
}
//! ===== TBD =====
// Steps to modify this method:
//   1. Check if socket has been started (handles, threads...)
//   2. If so, stop everything
//   3. Update _ip and _port attributed
//   4. Update configuration of the socket
//   5. If socket was previously started, restart the socket
//
// void Ethernet2::address(const std::string &ip, const uint16_t &port) {
//   if (this->is_loaded()) {
//     _ip   = ip;
//     _port = port;
//     SURFR_DEBUG(std::string("Socket IP:PORT set to ") + _ip + ":" + std::to_string(_port));
//   } else {
//     SURFR_DEBUG("Socket IP:PORT cannot be changed while started");
//   }
// }
//! ===== TBD =====

std::vector<std::string> Ethernet::commands() {
  std::vector<std::string> ret;

  for (Params::iterator it = _commands.begin(); it != _commands.end(); it++) {
    std::string kCmd = ((it->first).find('/') != std::string::npos) ? (it->first).substr(0, (it->first).find('/')) : (it->first);

    if (std::find(ret.begin(), ret.end(), kCmd) == ret.end()) {
      ret.push_back(kCmd);
    }
  }

  return ret;
}

uint32_t Ethernet::packet_size(const std::string& command) {
  uint32_t    ret = 0;
  std::string cmd = (command == "") ? _lastRequest : command;

  // 1. Checks if this command has an associated response
  if (_commands.find(cmd + "/response") != _commands.end()) {
  // 2. If so, checks if response packet size has been previously computed
    if (_packetSize.find(cmd) == _packetSize.end()) {
  // 2.1. If not, sums up the size of all the response parameters
      _packetSize[cmd] = 0;
      for (uint16_t k = 0; k < _commands[cmd + "/response"].size(); k++) {
        std::string kParam = _commands[cmd + "/response"][k];

        if (_commands.find(cmd + "/" + kParam) != _commands.end()) {
          XmlRpc::XmlRpcValue kValue  = _commands[cmd + "/" + kParam];
          std::string         kType   = std::string(kValue[0]);
          uint8_t             kSizeof = int(kValue[1]);
          uint32_t            kDim    = (kType.rfind("[") == std::string::npos) ? 1 :
                                        std::stoi(kType.substr(kType.rfind("[")+1, kType.size() - kType.rfind("]")));

          _packetSize[cmd] += kSizeof*kDim;
        } else {
          SURFR_ERROR("Parameter " + kParam + " not found in " + cmd + " response list.");
          _packetSize.clear();
          return ret;
        }
      }
      _buffer[cmd] = new uint8_t[_packetSize[cmd]]();
  // 2.2. Otherwise, returns the stored size
    }
    ret = _packetSize[cmd];
  }

  return ret;
}

bool Ethernet::_isResponse() {
  return (_packetSize.find(_lastRequest) != _packetSize.end());
}

uint32_t Ethernet::packet_count() {
  return _packetCount;
}

void Ethernet::_cbIoService() {
  _ioService.run();
}

void Ethernet::_cbTcpRead(const boost::system::error_code& err, std::size_t numbytes) {
  if (!err && this->is_running()) {
    {
//~ std::cout << "bytes received: " << numbytes << std::endl;
      boost::unique_lock<boost::mutex> lock(_mutex);
      boost::asio::async_read(*_tcpSocket,
                              boost::asio::buffer(_buffer[_lastRequest], this->packet_size() + ((BOOST_VERSION / 1000000 == 1) && ((BOOST_VERSION / 100) % 1000 >= 48))),
                              boost::bind(&Ethernet::_cbTcpRead,
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
      ++_packetCount;
      _condition.notify_all();
    }
  } else {
    SURFR_ERROR("Socket " << this->address() << " returned: [" << err.value() << "] " << err.message());
    this->stop();
  }
}

void Ethernet::_cbUdpRead() {
  try {
    while (this->is_running()) {
      if (this->_isResponse()) {
        size_t sz = _udpSocket->receive(boost::asio::buffer(_buffer[_lastRequest], this->packet_size() + 1));
        if (sz != this->packet_size()) {
          SURFR_WARNING("Received packet size (" << sz << " bytes) does not match expected size (" << this->packet_size() << " bytes)");
        } else {
          { 
            boost::unique_lock<boost::mutex> lock(_mutex);
            ++_packetCount;
            _condition.notify_all();
          }
        }
      }
    }
  } catch (std::exception &e) {
    SURFR_ERROR(e.what());
    this->stop();
  }
}

void Ethernet::input(Flow& data) {
    std::vector<uint8_t> packet;
    std::string          command = std::string(FLOW_CAST(data, XmlRpc::XmlRpcValue, "command"));
    XmlRpc::XmlRpcValue  request = surfr::unparse_params(_commands, command);
    this->packet_size(command);

if (request.hasMember("request")) {
    for (uint16_t k = 0; k < request["request"].size(); k++) {
      XmlRpc::XmlRpcValue kValue;

      if (request["request"][k].getType() == XmlRpc::XmlRpcValue::TypeString) {
        if (request.hasMember(std::string(request["request"][k]))) {
          kValue = FLOW_CAST(data, XmlRpc::XmlRpcValue, request["request"][k]);
// IMPORTANT NOTE: ACTUALLY, THE FORMAT OF THIS INPUT VALUE IS RESPONSABILITY OF THE USER.
//                 IN THE FUTURE, THIS CODE SHOULD CHECK THE FORMAT PROVIDED BY THE YAML
//                 FILE WITH THE INPUT DATA
        } else {
          kValue = request["request"][k];
        }
      } else {
        kValue = request["request"][k];
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
          // int kInt = int(kValue[i]);
          uint32_t kInt = abs(int(kValue[i]));
          while (kInt > 255) {
            kInt = kInt % 256;
            packet.push_back(kInt);
          }
          packet.push_back(kInt);
        }
      } else if (kValue.getType() == XmlRpc::XmlRpcValue::TypeString) {
        if (std::string(kValue) == "\n") {
          packet.push_back('\n');
        } else {
          for (uint32_t i = 0; i < kValue.size(); i++) {
            packet.push_back(std::string(kValue)[i]);
// std::cout << std::string(kValue)[i];
          }
        }
      } 
    }
// std::cout << std::endl;
    // _lastRequest = command;

  if (_protocol == PROTOCOL::UDP) {
    // Send the requested packet through socket
    _udpSocket->send(boost::asio::buffer(packet.data(), packet.size()));
  } else if (_protocol == PROTOCOL::TCP) {
    _tcpSocket->send(boost::asio::buffer(packet.data(), packet.size()));
  } else {
    SURFR_ERROR("Protocol undefined, cannot send data.");
  }
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
}

void Ethernet::output(Flow& data) {
if (this->is_running()) {
  if (this->_isResponse()) {
//~ size_t sz = 0;
//~ try {
//~ while (sz != this->packet_size()) {
  //~ // uint32_t warn_delay = 0;
  //~ switch(_protocol) {
  //~ case PROTOCOL::UDP:
    //~ sz = _udpSocket->receive(boost::asio::buffer(_buffer[_lastRequest], this->packet_size() + 1));
    //~ break;
  //~ case PROTOCOL::TCP:
    //~ sz = _tcpSocket->receive(boost::asio::buffer(_buffer[_lastRequest], this->packet_size() + 1));
//~ #if ((BOOST_VERSION / 1000000 == 1) && ((BOOST_VERSION / 100) % 1000 >= 48))
    //~ sz = _tcpSocket->receive(boost::asio::buffer(_buffer[_lastRequest], this->packet_size() + 1));
//~ #else
    //~ sz = _tcpSocket->receive(boost::asio::buffer(_buffer[_lastRequest], this->packet_size()));
//~ #endif
    //~ break;
  //~ }
  //~ if (sz != this->packet_size()) {
    //~ // if (warn_delay < this->hz()) {
    //~ //   warn_delay++;
    //~ // } else {
      //~ SURFR_DEBUG("Received packet size (" << sz << " bytes) does not match expected size (" << this->packet_size() << " bytes)");
    //~ // }
  //~ } else {
    //~ { 
      //~ boost::unique_lock<boost::mutex> lock(_mutex);
      //~ _packetCount++;
      //~ _condition.notify_all();
    //~ }
    //~ // warn_delay = 0;
  //~ }
//~ }
//~ } catch (std::exception &e) {
  //~ SURFR_ERROR(e.what());
  //~ this->stop();
//~ }
//~ if (sz == this->packet_size()) {

// Buffer is mutexed and updated asynchronously on a read callback thread
  uint8_t* kPtr = _buffer[_lastRequest];
  for (uint16_t kList = 0; kList < _commands[_lastRequest + "/response"].size(); kList++) {
    std::string         kParam = _commands[_lastRequest + "/response"][kList];

    if (_commands.find(_lastRequest + "/" + kParam) != _commands.end()) {
      XmlRpc::XmlRpcValue kValue  = _commands[_lastRequest + "/" + kParam];
      std::string         kType   = std::string(kValue[0]);
      uint8_t             kSizeof = int(kValue[1]);
      uint32_t            kDim    = (kType.rfind("[") == std::string::npos) ? 1 :
                                    std::stoi(kType.substr(kType.rfind("[")+1, kType.size() - kType.rfind("]")));

      if (kType.find("Unsigned") != std::string::npos) {
        uint32_t              kData1;
        std::vector<uint32_t> kDataN;
        for (uint32_t kIdx = 0; kIdx < kDim; kIdx++) {
          kData1 = 0;
          for (uint32_t k = 0, pow = 1; k < kSizeof; k++, pow *= 256) {
            kData1 += (*(kPtr + (kSizeof - 1 - k)))*pow;
          }
          kDataN.push_back(kData1);
          kPtr += kSizeof;
        }
        if (kDim == 1) {
          (_out)[kParam] = kData1;
        } else {
          (_out)[kParam] = kDataN;
        }
      } else if (kType.find("Integer") != std::string::npos) {
        int32_t              kData1;
        std::vector<int32_t> kDataN;
        for (uint32_t kIdx = 0; kIdx < kDim; kIdx++) {
          kData1 = 0;
          for (uint32_t k = 0, pow = 1; k < kSizeof; k++, pow *= 256) {
            kData1 += (*(kPtr + (kSizeof - 1 - k)))*pow;
            if ( (k == kSizeof - 1) && (kData1 >= 128*pow) ) {
              kData1 -= 256*pow;
            }
          }
          kDataN.push_back(kData1);
          kPtr += kSizeof;
        }
        if (kDim == 1) {
          (_out)[kParam] = kData1;
        } else {
          (_out)[kParam] = kDataN;
        }
      } else if (kType.find("Double") != std::string::npos) {
        union {
          double_t value;
          uint8_t  array[sizeof(double_t)];
        }                     kData1;
        std::vector<double_t> kDataN;
        for (uint32_t kIdx = 0; kIdx < kDim; kIdx++) {
          for (uint8_t k = 0; k < kSizeof; k++) {
            kData1.array[(kSizeof - 1) - k] = *(kPtr + k);
          }
          kDataN.push_back(kData1.value);
          kPtr += kSizeof;
        }
        if (kDim == 1) {
          (_out)[kParam] = kData1;
        } else {
          (_out)[kParam] = kDataN;
        }
      }
    } else {
      SURFR_ERROR("Parameter " + kParam + " not found in response list.");
    }
  }
//~ }
  }
}

  data = _out;
}
bool Ethernet::service(Flow& data) {
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
