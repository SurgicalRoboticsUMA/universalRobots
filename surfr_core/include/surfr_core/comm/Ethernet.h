/*
 *
 *  innlab_core/comm/Ethernet.h
 * 
 *               Ethernet Communication class
 *               ----------------------------------------------------------
 *  Begin Date : November 13, 2016
 *  Revision   : November 09, 2018 (rev 16)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Allow calling the configure() method outside the class
 *                 (now it is at the constructor).
 *               - Hot change of the IP:PORT address.
 *               - Use of try-catch-throw for exceptions handling.
 *               - Full review of Doxygen documentation.
 *  Done List  : - Added compatibility with older Boost versions below 1.48.
 *                 This enhances the use of Ethernet class with ROS Hydro (R16)
 *               - The thread for the response callback has been removed.
 *                 Now, socket responses are read directly on the output
 *                 method. Only tested for TCP socket (must check for UDP)
 *                 (R15)
 *               - When a packet received was not correctly read (i.e. wrong
 *                 packet size), corrupted data was sent anyway. Now, the
 *                 output method sends the last correct packet read. Only
 *                 tested for TCP socket (must check for UDP) (R14)
 *               - Input commands can now have the special tag "\n". If so,
 *                 it is translated as a return carriage (R13)
 *               - Fixed a problem when no request command is defined. Now
 *                 the input method updates the _lastRequest attribute (R12)
 *               - Ethernet class now supports both, TCP and UDP connections
 *                 as well as dual I/O communication (R11)
 *               - The formatting on the YAML now allows the setup of
 *                 multiple sockets (R10)
 *               - The formatting on the YAML file is now much easier to
 *                 follow (R9)
 *               - Now the class automatically formats the packets sent and
 *                 received from a template given from YAML file. Currently
 *                 this works only with UDP protocol (R8).
 *               - This new release enhances the use of UDP or TCP protocols.
 *                 Besides, all the code is being adapted to threads
 *                 philosophy. This version yet needs more debugging 
 *                 (specially the TCP protocol) and a lot of code
 *                 cleaning (R7)
 *               - Class has been adapted to allow its use with the ROS
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

// Surgical Framework include files
#include "surfr_core/Base.h"
// Here you may include whatever header files required to connect your device:
// C++ Ethernet socket include files
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
// Boost threads include files
#include <boost/version.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>


namespace surfr { namespace comm {
    /// Definition of communication protocols available
    typedef enum { NONE2 = 0,
                   UDP  = 1,
                   TCP  = 2
    } PROTOCOL;
    /// Constant values
    const std::map<PROTOCOL, std::string> PROTOCOL_DESC = { {PROTOCOL::NONE2, "NONE"},
                                                            {PROTOCOL::UDP , "UDP"},
                                                            {PROTOCOL::TCP , "TCP"} };

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
   *         * \b protocol -- can be "UDP" or "TCP" string.
   *         * \b ip -- the IP address of the Ethernet socket.
   *         * \b port -- the PORT channel of the Ethernet socket.
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
  /*! \brief Returns the current communication protocol of the Ethernet socket
   * 
   *  \return A \b std::string value which tells connection mode: \a "UDP"
   *          for User Datagram Protocol, \a "TCP" for Transmission
   *          Control Protocol.
   *  \sa address(), size(), debug()
   */
    std::string protocol();
  /*! \brief Sets a new communication protocol of the Ethernet socket
   * 
   *  Please note that this method will not update the communication protocol
   *  if socket is already configured. If so, class must be cleaned up first.
   * 
   *  \param protocol is a \b PROTOCOL value which tells connection protocol:
   *         PROTOCOL::UDP for User Datagram Protocol, \a PROTOCOL::TCP for
   *         Transmission Control Protocol.
   *  \sa start(), address(), size(), debug()
   */
    void protocol(PROTOCOL protocol);
  /*! \brief Returns the current IP of the Ethernet socket
   * 
   *  \return A string value which tells the IP of the Ethernet
   *          socket (i.e. 192.168.0.1).
   *  \sa protocol(), address(), port(), debug()
   */
    std::string ip();
  /*! \brief Returns the current PORT of the Ethernet socket
   * 
   *  \return A uint16_t value which tells the PORT of the Ethernet socket.
   *  \sa protocol(), address(), ip(), debug()
   */
    uint16_t    port();
  /*! \brief Returns the current IP:PORT address of the Ethernet socket
   * 
   *  \return A string value which tells the full address of the Ethernet
   *          socket with format IP:PORT (i.e. 192.168.0.1:80).
   *  \sa protocol(), ip(), port(), debug()
   */
    std::string address();
std::vector<std::string> commands();
  /*! \brief Returns the packet size in bytes sent through the Ethernet socket
   * 
   *  \return A uint32_t value with the size of the requested packet, in bytes.
   *  \sa protocol(), address()
   */
    uint32_t packet_size(const std::string& command = "");
    uint32_t packet_count();

    void  output(surfr::Flow& data);
    void  input(surfr::Flow& data);
    bool  service(surfr::Flow& data);
  private:
    std::map<std::string, uint8_t*> _buffer;                                    /*!< Memory space to save packets received from socket.*/
    PROTOCOL                        _protocol;                                  /*!< Set to work with UDP/TCP protocol.                */
    std::string                     _ip;                                        /*!< IP address of the socket opened.                  */
    uint16_t                        _port;                                      /*!< PORT address of the socket opened.                */
    uint32_t                        _packetCount;                               /*!< Number of packets received from socket.           */
    std::map<std::string, uint32_t> _packetSize;                                /*!< Size in bytes of each command response.           */
    std::string                     _lastRequest;                               /*!< Last requested command.                           */
    Params                          _commands;                                  /*!< List of available commands for the socket.        */

    boost::asio::io_service         _ioService;
    boost::thread                   _thread;
    boost::mutex                    _mutex;
    boost::condition                _condition;
    boost::asio::ip::udp::socket*   _udpSocket;
    boost::asio::ip::tcp::socket*   _tcpSocket;
    boost::asio::ip::tcp::resolver* _tcpResolver;

    /// This method starts the IO service to get socket packets asynchronously
    void _cbIoService();
    void _cbTcpRead (const boost::system::error_code& err, std::size_t numbytes);
    bool _isResponse();
    void _cbUdpRead ();
  };

} // namespace comm
} // namespace surfr

#endif  //_SURFR_CORE_ETHERNET_H
