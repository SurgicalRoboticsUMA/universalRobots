/*
 *
 *  surfr_urx/DeviceURx.h
 * 
 *               Universal Robots Device class
 *               ----------------------------------------------------------
 *  Begin Date : September 2, 2016
 *  Revision   : January 17, 2019 (rev 23)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Update the StateMachine class with a new state to check
 *                 if class has been initialized, as constructor must now
 *                 be called through "init" method.
 *  Done List  : - Added support for multiple Polyscope versions (from 3.0 to
 *                 3.8). Each version has its own Ethernet protocol, which can
 *                 be selected by an input argument on the launch file (R23)
 *               - Cleaning code (R22)
 *               - Services has been fully adapted to the new Ethernet
 *                 class (R21)
 *               - Now the class can receive service requests with the new
 *                 version of the Ethernet class (R20)
 *               - The class has been adapted to accept the new version
 *                 of the Ethernet class, which accepts the protocol
 *                 description by means of a YAML file. Yet, only the
 *                 reception of data is accepted with such class. First
 *                 tests show a high stability on communications (R19)
 *               - The GUI socket has been deprecated and disabled (R18)
 *               - Now the class handles the connection errors. In this
 *                 case the class is automatically initialized on
 *                 simulation mode (R17)
 *               - Fixed input() and service() methods (R16)
 *               - Class has been adapted to allow its use with the ROS
 *                 pluginlib package. This enchances the use of only one
 *                 unique main program which can dynamically run any
 *                 device (R15)
 *               - Improved integration with StateMachine class (R14)
 *               - Fixed behavior and state update when a socket is not
 *                 correctly started (R13).
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


#if !defined(_SURFR_DEVICEURX_H)
#define _SURFR_DEVICEURX_H

#include <surfr_core/Base.h>
#include <surfr_core/comm/Ethernet.h>


namespace surfr { namespace dev {
/*! \brief Class for the Universal Robots device driver
 * 
 * This class communicates with the Universal Robots manipulators
 * through ethernet for accessing to the available services.
 * 
 * 
 */
  class DeviceURx : public surfr::Base {
  public:
  /*! \brief Constructor of the UniversalRobotsDriver class
   */
    DeviceURx();
  /*! \brief Destructor of the UniversalRobotsDriver class
   */
    virtual ~DeviceURx();
  /*! \brief Custom configuration of the Universal Robots device
   * 
   * 
   *  \return A bool value which tells if the Universal Robots device is
   *          successfully configured.
   *  \sa start(), stop()
   */
    bool configureHook();
  /*! \brief Custom start of the communication protocol for Universal Robots
   * 
   *  \return A bool value which tells if the Universal Robots device is
   *          successfully started.
   *  \sa configure(), stop()
   */
    bool startHook();
  /*! \brief Custom stop of the communication protocol for Universal Robots
   * 
   * 
   *  \return A bool value which tells if the Universal Robots device is
   *          successfully stopped.
   *  \sa start(), configure()
   */
    bool stopHook();

    void output (surfr::Flow& data);
    void input  (surfr::Flow& data);
    bool service(surfr::Flow& data);
  private:
    surfr::comm::Ethernet* _socketIn;
    surfr::comm::Ethernet* _socketOut;
  };

} // namespace dev
} // namespace surfr

#endif  //_SURFR_DEVICEURX_H
