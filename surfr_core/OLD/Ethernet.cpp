/*
 *
 *  surfr_core/comm/Ethernet.cpp
 * 
 *               Ethernet Communication class
 *               ----------------------------------------------------------
 *  Begin Date : November 13, 2016
 *  Revision   : October 25, 2017 (rev 6)
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
_handle(0) {
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
  if (_buffer)            delete _buffer;
  if (_tcpResolver)       delete _tcpResolver;
  if (_tcpSocket)         delete _tcpSocket;

  SURFR_DEBUG("Socket " + this->address() + " destroyed.");
}

// bool Ethernet::configureHook(const surfr::Params& cfg) {
bool Ethernet::configureHook() {
  const surfr::Params cfg = _config;
/*
 * Creation of OUTPUT parameters
 */
  (_out)["simulation"] = bool(_is_simulation);
  (_out)["data"]       = std::string("");
  
/*
 * Creation of the communication handles and buffer
 */
  if (cfg.find("ip") != cfg.end()) {
    _ip = std::string((XmlRpc::XmlRpcValue) cfg.at("ip"));
    SURFR_DEBUG("IP: " + _ip);
  } else {
    SURFR_ERROR("Undefined IP address");
    return false;
  }
  if (cfg.find("port") != cfg.end()) {
    _port = std::string((XmlRpc::XmlRpcValue) cfg.at("port"));
    SURFR_DEBUG("Port: " + _port);
  } else {
    SURFR_ERROR("Undefined Port address");
    return false;
  }
  if (cfg.find("packet_size") != cfg.end()) {
    _packetSize = int((XmlRpc::XmlRpcValue) cfg.at("packet_size"));
    SURFR_DEBUG("Packet size: " + std::to_string(_packetSize));
  } else {
    _packetSize = DEFAULT_PACKET_SIZE;
    SURFR_WARNING("Packet size not found, assigned default value (" + std::to_string(_packetSize) + " bytes)");
  }
  std::string tmpMode = "input";
  if (cfg.find("mode") != cfg.end()) {
    tmpMode = std::string((XmlRpc::XmlRpcValue) cfg.at("mode"));
    SURFR_DEBUG("Communication mode: " + tmpMode);
  } else {
    SURFR_ERROR("Communication mode not found");
    return false;
  }
  if        (!tmpMode.compare("output" )) {
    _mode = Mode::SEND;
  } else if (!tmpMode.compare("input")) {
    _mode = Mode::RECV;
  } else if (!tmpMode.compare("dual"  )) {
    _mode = Mode::DUAL;
  } else {
    _mode = Mode::NONE;
    SURFR_WARNING("Invalid communication mode!");
  }
  if (_mode != Mode::NONE) {
    SURFR_DEBUG("Communication mode: " + MODE_DESC.at(_mode));
  }

  _buffer      = new char[_packetSize];
  _tcpResolver = new tcp::resolver(_ioService);
  _tcpSocket   = new tcp::socket(_ioService);

  SURFR_INFO("Socket " + this->address() + " configured.");
  return true;
}

bool Ethernet::startHook() {
  bool ret = false;

  switch (_mode) {
 /*
  * Opening the INPUT socket to receive commands from the device 
  */
  case Mode::RECV: {
    tcp::resolver::query tcpQuery(_ip, _port);
    _tcpResolver->async_resolve(tcpQuery,
                                boost::bind(&Ethernet::_cbResolve,
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::iterator));
    // This thread keeps the I/O service running in parallel
    boost::thread(&Ethernet::_cbIoService, this);

    SURFR_INFO("Socket " + this->address() + " opened to RECEIVE data");
    ret = true;
    break; }
 /*
  * Opening the OUTPUT socket to send commands to the device 
  */
  case Mode::SEND: {
    int    err;                                                                                   // Error identifier
    int    socketOptions = 1;                                                                     // Socket options flag
    struct sockaddr_in socketAddress;                                                             // Socket address setup
      socketAddress.sin_family = AF_INET;                                                         // Connect mode
      socketAddress.sin_port   = htons(atoi(_port.c_str()));                                      // Port
    inet_aton(_ip.c_str(), &(socketAddress.sin_addr));                                            // Address
    memset(&(socketAddress.sin_zero), '\0', 8);                                                   // End char

    //! NOTE: Use SOCK_STREAM for TCP and SOCK_DGRAM for UDP
    // handle = socket(AF_INET, SOCK_STREAM, 0);
    _handle = socket(AF_INET,
                     SOCK_STREAM,
                     IPPROTO_TCP);
    if (_handle < 0) {
      SURFR_ERROR(strerror(errno));
      break;
    }

    err = connect(_handle,
                  (const sockaddr*) &socketAddress,
                  sizeof(struct sockaddr_in));
    if (err < 0) {
      SURFR_ERROR("Socket " + this->address() + " returned: " + std::to_string(errno) + " " + strerror(errno));
      break;
    } 

    err = setsockopt(_handle,
                     IPPROTO_TCP,
                     TCP_NODELAY || TCP_QUICKACK,
                     (char*) &socketOptions,
                     sizeof(int));
    if (err < 0) {
      SURFR_ERROR(strerror(errno));
      break;
    }

    SURFR_INFO("Socket " + this->address() + " opened to SEND data");
    ret = true;
    break; }
  default:
    SURFR_DEBUG("Socket mode not defined, nothing done");
  }
  
  if (ret) {
    SURFR_INFO("Socket " + this->address() + " started.");
  }
  return ret;
}

bool Ethernet::stopHook()
{
 /*
  * Close all opened socket handles
  */
  _tcpSocket->close();
  close(_handle);

  SURFR_INFO("Socket " + this->address() + " stopped.");
  return true;
}

Mode Ethernet::mode() {
  return _mode;
}
void Ethernet::mode(Mode mode) {
  if (this->is_loaded()) {
    _mode = mode;
    SURFR_DEBUG(std::string("Socket Mode set to Mode::") + MODE_DESC.at(_mode));
  } else {
    SURFR_DEBUG(std::string("Socket Mode can only be set at State::") + STATE_DESC.at(surfr::STATE_LOADED));
  }
}

std::string Ethernet::address() {
  return (_ip + ":" + _port);
}
std::string Ethernet::ip() {
  return _ip;
}
uint32_t Ethernet::port() {
  return uint32_t(atoi(_port.c_str()));
}
void Ethernet::address(const std::string &ip, const std::string &port) {
  if (this->is_loaded()) {
    _ip   = ip;
    _port = port;
    SURFR_DEBUG(std::string("Socket IP:PORT set to ") + _ip + ":" + _port);
  } else {
    SURFR_DEBUG("Socket IP:PORT cannot be changed while started");
  }
}

unsigned int Ethernet::size() {
  return _packetSize;
}
void Ethernet::size(unsigned int sz) {
  if (this->is_loaded()) {
    _packetSize = sz;
    SURFR_DEBUG(std::string("New packet size set to ") + std::to_string(_packetSize));
  } else {
    SURFR_DEBUG("Socket PACKET SIZE cannot be changed while started");
  }
}

char* Ethernet::recv() {
  return _buffer;
}

void Ethernet::send(char* packet) {
  std::string c_packet(packet);
  c_packet = c_packet + "\n";
  if (write(_handle, c_packet.c_str(), strlen(c_packet.c_str())) < 0) {
    SURFR_ERROR(strerror(errno));
  } else {
    SURFR_DEBUG(std::string(packet));
  }
}
void Ethernet::send(const char* packet) {
  this->send((char *) packet);
}

void Ethernet::_cbResolve(const boost::system::error_code& err, tcp::resolver::iterator it)
{
  // Attempt a connection to the first endpoint in the list. Each endpoint will
  // be tried until we successfully establish a connection.
  if (!err) {
    tcp::endpoint endPoint = *it;
    _tcpSocket->async_connect(endPoint,
                              boost::bind(&Ethernet::_cbConnect,
                                          this,
                                          boost::asio::placeholders::error,
                                          ++it));
  } else {
    std::stringstream msg;
    msg << err.value() << ": " << err.message();
    SURFR_ERROR("Socket " + this->address() + " returned: " + msg.str());
    this->stop();
  }
}

void Ethernet::_cbConnect(const boost::system::error_code& err, tcp::resolver::iterator it)
{
  if (!err) {
    boost::asio::async_read(*_tcpSocket,
                            boost::asio::buffer(_buffer, _packetSize),
                            boost::bind(&Ethernet::_cbRead,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
  } else {
    // std::stringstream msg;
    // msg << err.value() << ": " << err.message();
    // SURFR_ERROR("Socket " + this->address() + " returned: " + msg.str());
    this->stop();
  }
}

void Ethernet::_cbRead(const boost::system::error_code& err, std::size_t numbytes)
{
  if (!err) {
    boost::asio::async_read(*_tcpSocket,
                            boost::asio::buffer(_buffer, _packetSize),
                            boost::bind(&Ethernet::_cbRead,
                                        this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
  } else {
    std::stringstream msg;
    msg << err.value() << ": " << err.message();
    SURFR_ERROR("Socket " + this->address() + " returned: " + msg.str());
  }
}

void Ethernet::_cbIoService()
{
  _ioService.run();
}

void Ethernet::input(Flow& data) {
  // const char *command = (FLOW_CAST(data, std::string, "data")).c_str();
  this->send((FLOW_CAST(data, std::string, "data")).c_str());
  SURFR_DEBUG("Input: " << (FLOW_CAST(data, std::string, "data")).c_str());
}
void Ethernet::output(Flow& data) {
  (_out)["data"] = std::string(_buffer, _buffer + _packetSize);
  //~ std::cout << FLOW_CAST(_out, std::string, "data") << "<EOL>"<< std::endl;

  data = _out;
}
bool Ethernet::service(Flow& data) {
  // Service message
  std::string srvName = FLOW_CAST(data, std::string, "service");
  std::string srvType = FLOW_CAST(data, std::string, "type");
  std::string srvData = std::string(FLOW_CAST(data, XmlRpc::XmlRpcValue, "data"));

  if (boost::iequals("string", srvType)) {
    SURFR_DEBUG("Service " << srvName << " sending packet: " + srvData);  
    this->send(srvData.c_str());
  } else {
    SURFR_WARNING("Service " << srvName << " did not receive a string type");
  }

  return true;
}

} // namespace comm
} // namespace surfr
