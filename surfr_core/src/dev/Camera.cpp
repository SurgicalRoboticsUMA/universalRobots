/*
 *
 *  surfr_core/dev/Camera.cpp
 * 
 *               Universal Robots Device class
 *               ----------------------------------------------------------
 *  Begin Date : September 2, 2016
 *  Revision   : October 19, 2017 (rev 14)
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


#include "surfr_core/dev/Camera.h"


namespace surfr { namespace dev {

Camera::Camera(const Params& cfg, uint32_t id):
  Base::Base(cfg, id) {
  // _socket(NULL) {
  if (!this->ok()) {
    SURFR_ERROR("DeviceBase not correctly initialized, constructor aborted.");
    return;
  }
  // this->configure(_config);
  SURFR_DEBUG("Done");
}

Camera::~Camera() {
  /*
   * Destruction of the communication handles
   */
    if (this->is_running()) this->stop();
    // if (_socket)            delete _socket;
  
    SURFR_DEBUG("Done");
}
  
// bool Camera::configureHook(const surfr::Params& cfg) {
bool Camera::configureHook() {
/*
 * Creation of OUTPUT parameters
 */
 
cv::VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
 
if (!stream1.isOpened()) { //check if video device has been initialised
std::cout << "cannot open camera" << std::endl;
}
 
//unconditional loop
while (true) {
cv::Mat cameraFrame;
stream1.read(cameraFrame);
cv::imshow("cam", cameraFrame);
if (cv::waitKey(30) >= 0)
break;
}

/*
 * Creation of the communication parameters
 */
  // Params _config;
  //   _config["mode"]                        = XmlRpc::XmlRpcValue("input");
  //   _config["frequency"]                   = cfg.at("frequency");
  //   _config["models/Ethernet/description"] = XmlRpc::XmlRpcValue("Ethernet socket to IP camera");

  // if (cfg.find("sockets/ip") != cfg.end()) {
  //   _config["ip"] = cfg.at("sockets/ip");
  // } else {
  //   SURFR_ERROR("Socket IP not found");
  //   _state = surfr::STATE_NONE;
  //   return false;
  // }
  // if (cfg.find("sockets/packet_size") != cfg.end()) {
  //   _config["packet_size"] = cfg.at("sockets/packet_size");
  // } else {
  //   SURFR_ERROR("Packet size value not found");
  //   _state = surfr::STATE_NONE;
  //   return false;
  // }
  // if (cfg.find("sockets/port") != cfg.end()) {
  //   _config["port"] = cfg.at("sockets/port");
  // } else {
  //   SURFR_ERROR("Output port value not found");
  //   _state = surfr::STATE_NONE;
  //   return false;
  // }

  // SURFR_DEBUG("Socket Parameters:");
  // SURFR_DEBUG("  Description       : " + std::string(_config.at("models/Ethernet/description")));
  // SURFR_DEBUG("  Communication mode: " + std::string(_config.at("mode")));
  // SURFR_DEBUG("  Socket            : " + std::string(_config.at("ip")) + ":" + std::string(_config.at("port")));
  // SURFR_DEBUG("  Packet size       : " + std::to_string((int) _config.at("packet_size")));
  // SURFR_DEBUG("  Frequency         : " + std::to_string((int) _config.at("frequency")));

/*
 * Creation of the communication handles
 */
  // _socket  = new surfr::comm::Ethernet(_config, 1);

  SURFR_INFO("Camera labeled as " + this->name() + " has been configured.");
  return true;
}

bool Camera::startHook() {
/*
 * Start of the communication parameters
 */
  // if (!_socket->start()) {
  //   SURFR_ERROR("IP socket could not be started, aborting...");
  //   _state = surfr::STATE_EXCEPTION;
  //   return false;
  // }

  return true;
}

bool Camera::stopHook()
{
 /*
  * Close opened socket handle
  */
  // _socket->stop();

  SURFR_INFO("Camera labeled as " + this->name() + " has been stopped.");
  return true;
}

void Camera::output(Flow& data) {
  // if (_socket) {
  // }
  data = _out;
}

void Camera::input(Flow& data) {
}

/*
 * NOTA: Corregir esta funcion para que reciba como datos:
 * - Comando
 * - NumParams
 * - ArrayParams
 * De esta forma se puede hacer un servicio generico para toda la libreria del UR3
 */
void Camera::service(Flow& data) {
}

} // namespace dev
} // namespace surfr
