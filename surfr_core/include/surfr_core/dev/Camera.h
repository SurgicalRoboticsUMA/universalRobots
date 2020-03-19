/*
 *
 *  surfr_core/dev/Camera.h
 * 
 *               Generic Camera device class
 *               ----------------------------------------------------------
 *  Begin Date : September 2, 2016
 *  Revision   : October 19, 2017 (rev 1)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : 
 *  Done List  : - Initial release (R1)
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


#if !defined(_SURFR_CORE_CAMERA_H)
#define _SURFR_CORE_CAMERA_H

#include <surfr_core/Base.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <surfr_core/comm/Ethernet.h>


namespace surfr { namespace dev {
  /*! \brief Class for the Generic Camera device driver
   * 
   * This class communicates with a Generic WebCam device for
   * accessing to the available services.
   * 
   * 
   */
  class Camera : public surfr::Base {
  public:
    /*! \brief Constructor of the UniversalRobotsDriver class
       */
    Camera(const surfr::Params& cfg, uint32_t id = 0);
    
    /*! \brief Destructor of the UniversalRobotsDriver class
     */
    ~Camera();
    /*! \brief Custom configuration of the Universal Robots device
     * 
     * 
     *  \return A bool value which tells if the Universal Robots device is
     *          successfully configured.
     *  \sa start(), stop()
     */
    // bool configureHook(const surfr::Params& cfg);
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
    void service(surfr::Flow& data);
  private:
    // surfr::comm::Ethernet* _socket;
  };

} // namespace dev
} // namespace surfr

#endif  //_SURFR_CORE_CAMERA_H
