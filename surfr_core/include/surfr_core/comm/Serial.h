/*
 *
 *  innlab_core/comm/Serial.h
 * 
 *               Serial Port Communication class
 *               ----------------------------------------------------------
 *  Begin Date : September 11, 2018
 *  Revision   : January 29, 2019 (rev 1)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Allow calling the configure() method outside the class
 *                 (now it is at the constructor).
 *               - Hot change of the Serial PORT.
 *               - Use of try-catch-throw for exceptions handling.
 *               - Full review of Doxygen documentation.
 *  Done List  : - Basic functionality of Serial Port. Class constructed
 *                 by using the Ethernet class as template.
 *                 IMPLEMENTED BUT NOT TESTED YET!!! (R1)
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

#if !defined(_SURFR_CORE_SERIAL_H)
#define _SURFR_CORE_SERIAL_H

// Surgical Framework include files
#include "surfr_core/Base.h"
// Here you may include whatever header files required to connect your device:
// Boost threads include files
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>


namespace surfr { namespace comm {
  /*! \brief Class for the Communication Serial Port driver
   * 
   * This class creates an asynchronous Communication via Serial
   * Port protocol. It only supports communications in one
   * direction at a time: receiving or sending data.
   * 
   */
  class Serial : public surfr::Base {
  public:
  /*! \brief Constructor of the Serial Port communication class
   * 
   * This constructor initializes the Serial Port
   *
   */
    Serial();
  /*! \brief Initialization of the Serial Port Communication class
   * 
   *  \param cfg is a surfr::Params map with the following parameters:
   *         * \b baudrate -- Speed of the Serial Communication
   *         * \b port -- the PORT channel of the Serial Communication.
   *  \param id is an unsigned integer value with a unique identifier
   *         label for the object. By default this value will be 0.
   */
    void init(surfr::Params& cfg, unsigned int id = 0);
  /*! \brief Destructor of the Serial Port Communication class
   */
    virtual ~Serial();
  /*! \brief Custom configuration of the Serial Port
   * 
   * 
   *  \return A bool value which tells if the Serial Port is
   *          succesfully configured.
   *  \sa start(), stop()
   */
    bool configureHook();
  /*! \brief Custom start of the communication protocol for a Serial Port
   * 
   *  \return A bool value which tells if the Serial Port is successfully
   *          started.
   *  \sa start(), stop()
   */
    bool startHook();
  /*! \brief Custom stop of the communication protocol for a Serial Port
   * 
   * 
   *  \return A bool value which tells if the Serial Port is successfully
   *          stopped.
   *  \sa start(), configure()
   */
    bool stopHook();
  /*! \brief Returns the current PORT of the Serial Port
   * 
   *  \return A std::string value which tells the PORT of the Serial Port.
   *  \sa debug()
   */
    std::string port();
uint32_t baud();
void baud(const uint32_t& rate);
std::vector<std::string> commands();
  /*! \brief Returns the packet size in bytes sent through the Ethernet socket
   * 
   *  \return A uint32_t value with the size of the requested packet, in bytes.
   *  \sa protocol(), address()
   */
    //~ uint32_t packet_count();

    void  output(surfr::Flow& data);
    void  input(surfr::Flow& data);
    bool  service(surfr::Flow& data);
  private:
    uint32_t                        _baudrate;                                            /*!< Serial Port baud rate.                 */
    std::string                     _port;                                                /*!< Serial Port name.                      */
    boost::asio::io_service         _ioService;                                           /*!< Internal communication.                */
    boost::asio::serial_port*       _handle;                                              /*!< Serial port handle.                    */
    std::map<std::string, uint8_t*> _buffer;                                              /*!< Memory space to save packets received. */
    //~ uint32_t                        _packetCount;                                         /*!< Number of packets received.            */
    std::string                     _lastRequest;                                         /*!< Last requested command.                */
    Params                          _commands;                                            /*!< List of available commands.            */
    bool                            _isResponse;

    /*! \brief Data reception from serial port.
     * 
     * String received from the serial connection after sending a command.
     * 
     * \param data is the returned read line from the serial communication port.
     * \returns true if reception protocol went ok, false otherwise.
     */
    bool recv(std::string& data);
    /*! \brief Command sent towards serial port.
     * 
     * Send a string command towards the serial connection to the device.
     * 
     * \param cmd is the requested command to the device.
     * \returns true if command sending went ok, false otherwise.
     */
    bool send(std::string cmd);
  };

} // namespace comm
} // namespace surfr

#endif  //_SURFR_CORE_SERIAL_H
