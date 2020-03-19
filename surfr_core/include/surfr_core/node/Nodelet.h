/*
 *
 *  surfr_core/node/Nodelet.h
 * 
 *               Generic nodelet class to load and link a device with ROS
 *               ----------------------------------------------------------
 *  Begin Date : October 18, 2017
 *  Revision   : October 25, 2017 (rev 3)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : 
 *  Done List  : - Class has been adapted to allow its use with the ROS
 *                 pluginlib package. This enchances the use of only one
 *                 unique main program which can dynamically run any
 *                 device (R3)
 *               - The nodelet has been adapted to initialize this custom
 *                 device (R2)
 *               - Initial release, creating nodelet template (R1)
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

#if !defined(_SURFR_CORE_NODELET_H)
#define _SURFR_CORE_NODELET_H


// ROS include files
#include <nodelet/nodelet.h>
// Boost threads include files
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
// Surgical Framework include files
#include "surfr_core/node/Node.h"
#include "surfr_core/node/class_list_macros.h"


namespace surfr
{
  class Nodelet : public nodelet::Nodelet {
  public:
    Nodelet();
    ~Nodelet();
    virtual void onInit();
    virtual void controller_poll();
  private:
    ros::NodeHandle*                     _nh;
    pluginlib::ClassLoader<surfr::Base>* _base;                                 // Pointer to plugin class loader.
    boost::shared_ptr<surfr::Base>       _dev;                                  // Pointer to device object.
    boost::shared_ptr<Node>              _node;                                 // Pointer to the node object.
    boost::shared_ptr<boost::thread>     _thread;                               // Pointer to the Node loop thread.
  };

} // namespace surfr


#endif  //_SURFR_CORE_NODELET_H
