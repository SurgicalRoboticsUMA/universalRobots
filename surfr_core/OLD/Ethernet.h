/*
 *
 *  innlab_core/comm/Ethernet.h
 * 
 *               Ethernet Communication class
 *               ----------------------------------------------------------
 *  Begin Date : November 13, 2016
 *  Revision   : October 25, 2017 (rev 6)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Allow calling the configure() method outside the class
 *                 (now it is at the constructor).
 *               - Set the dual read/write mode on a socket.
 *               - Use of try-catch-throw for exceptions handling.
 *               - Full review of Doxygen documentation.
 *  Done List  : - Class has been adapted to allow its use with the ROS
 *                 pluginlib package (R6)
 *               - Limit the set of IP, port and communication mode to
 *                 the loaded state (R5)
 *               - Customization of triggers by Hook() methods (R4)
 *               - Addition of comments (R3)
 *               - Configuration parameters can be send as XmlRpc value (R2)
 *               - Basic functionality of Ethernet for asynchronous
 *                 communication on read and write modes (R1)
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

#if !defined(_SURFR_CORE_ETHERNET_H)
#define _SURFR_CORE_ETHERNET_H

// C++ Ethernet socket include files
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
// Boost threads include files
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
// Surgical Framework include files
#include "surfr_core/comm/comm.h"
#include "surfr_core/Base.h"


namespace surfr { namespace comm {
  using boost::asio::ip::tcp;

  /// Constant values
  const int DEFAULT_PACKET_SIZE = 512;

  /*! \brief Class for the Communication Ethernet driver
   * 
   * This class creates an asynchronous Communication socket via
   * Ethernet TCP protocol. It only supports communications in one
   * direction, receiving or sending data.
   * 
   */
  class Ethernet : public surfr::Base {
  public:
  /*! \brief Constructor of the Ethernet Communication class
   */
    Ethernet();
  /*! \brief Initialization of the Ethernet Communication class
   * 
   *  \param cfg is a surfr::Params map with the following parameters:
   *         * \b mode -- connection mode: \a COMM_RECV for reading data
   *           from socket, \a COMM_SEND for sending data to socket and
   *           \a COMM_NONE if not set yet.
   *         * \b ip -- the IP address of the Ethernet socket.
   *         * \b port -- the PORT channel of the Ethernet socket.
   *         * \b packetSize -- the size, in bytes, of each packet sent
   *           through the socket. This parameter is not mandatory for
   *           \a COMM_SEND communications. By default it is \b 512.
   *
   *  \param id is an unsigned integer value with a unique identifier
   *         label for the object. By default this value will be 0.
   */
    void init(surfr::Params& cfg, unsigned int id = 0);
  /*! \brief Destructor of the Ethernet Communication class
   */
    virtual ~Ethernet();
  /*! \brief Custom configuration of the Ethernet socket
   * 
   * 
   *  \return A bool value which tells if the Ethernet socket is successfully
   *          configured.
   *  \sa start(), stop()
   */
    bool configureHook();
  /*! \brief Custom start of the communication protocol for a Ethernet socket
   * 
   *  \return A bool value which tells if the Ethernet socket is successfully
   *          started.
   *  \sa start(), stop()
   */
    bool startHook();
  /*! \brief Custom stop of the communication protocol for a Ethernet socket
   * 
   * 
   *  \return A bool value which tells if the Ethernet socket is successfully
   *          stopped.
   *  \sa start(), configure()
   */
    bool stopHook();
  /*! \brief Returns the current communication mode of the Ethernet socket
   * 
   *  \return A \b Mode value which tells connection mode: \a Mode::RECV
   *          for reading data from socket, \a Mode::SEND for sending data
   *          to socket and \a Mode::NONE if not set yet.
   *  \sa address(), size(), debug()
   */
    Mode mode();
  /*! \brief Sets a new communication mode of the Ethernet socket
   * 
   *  Please note that this method will not update the communication mode if
   *  socket is already configured. If so, class must be cleaned up first.
   * 
   *  \param mode is a \b Mode value which tells connection mode:
   *         \a Mode::RECV for reading data from socket, \a Mode::SEND for
   *         sending data to socket and \a Mode::NONE if not set yet.
   *  \sa start(), address(), size(), debug()
   */
    void mode(Mode mode);
  /*! \brief Returns the current IP:PORT address of the Ethernet socket
   * 
   *  \return A string value which tells the full address of the Ethernet
   *          socket with format IP:PORT (i.e. 192.168.0.1:80).
   *  \sa mode(), size(), debug()
   */
    std::string address();
    std::string ip();
    uint32_t    port();
  /*! \brief Sets a new IP:PORT address of the Ethernet socket
   * 
   *  Please note that this method will not update the communication mode if
   *  socket is already configured. If so, class must be cleaned up first.
   * 
   *  \param ip is a string value which tells IP address of the Ethernet
   *         socket.
   *  \param socket is a string value which tells PORT address of the
   *         Ethernet socket.
   *  \sa start(), mode(), size()
   */
    void address(const std::string &ip, const std::string &port);
  /*! \brief Returns the packet size in bytes sent through the Ethernet socket
   * 
   *  \return A unsigned integer value with the size of each packet, in bytes.
   *  \sa mode(), address()
   */
    unsigned int size();
  /*! \brief Sets the packet size in bytes sent through the Ethernet socket
   * 
   *  Please note that this method will not update the communication mode if
   *  socket is already configured. If so, class must be cleaned up first.
   * 
   *  \param sz is a unsigned integer value with the size of each packet,
   *         in bytes.
   *  \sa start(), mode(), address()
   */
    void  size(unsigned int sz);

    // boost::asio::io_service* io_ptr() { return &_ioService; };
    void  send(char* packet);
    void  send(const char* packet);
    char* recv();

    void  output(surfr::Flow& data);
    void  input(surfr::Flow& data);
    bool  service(surfr::Flow& data);
  private:
    Mode                    _mode;
    char*                   _buffer;
    int                     _packetSize;
    int                     _handle;
    std::string             _ip;
    std::string             _port;
    boost::asio::io_service _ioService;
    tcp::resolver*          _tcpResolver;
    tcp::socket*            _tcpSocket;

    void _cbResolve(const boost::system::error_code&, tcp::resolver::iterator);
    void _cbConnect(const boost::system::error_code&, tcp::resolver::iterator);
    void _cbRead   (const boost::system::error_code&, std::size_t);
    void _cbIoService();
  };

} // namespace comm
} // namespace surfr

#endif  //_SURFR_CORE_ETHERNET_H
