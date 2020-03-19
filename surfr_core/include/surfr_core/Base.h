/*
 *
 *  surfr_core/Base.h
 * 
 *               Base class for Surgical Framework devices
 *               ----------------------------------------------------------
 *  Begin Date : November 14, 2016
 *  Revision   : March 23, 2018 (rev 8)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Update the StateMachine class with a new state to check
 *                 if class has been initialized, as constructor must now
 *                 be called through "init" method.
 *               - Analysis of connection delay/heartbeat.
 *               - Use of try-catch-throw for exceptions handling.
 *               - Full review of Doxygen documentation.
 *  Done List  : - Making destructor a virtual method now lets a base
 *                 pointer to correctly call the destructor of the derived
 *                 class (R8)
 *               - Class has been adapted to allow its use with the ROS
 *                 pluginlib package (R7)
 *               - Class rewritten to match with the Device base class (R6)
 *               - Check for additional communication protocol parameters
 *                 (sampling, model...) (R5)
 *               - Partial addition of comments (R4)
 *               - Integration of State Machine class (R3)
 *               - Definition of virtual workflow I/O methods (R2)
 *               - Move code to cpp file (R1)
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

#if !defined(_SURFR_CORE_BASE_H)
#define _SURFR_CORE_BASE_H


#include "surfr_core/core.h"
#include "surfr_core/StateMachine.h"
//~ #include "surfr_core/node/class_list_macros.h"
#include <pluginlib/class_list_macros.h>
// NOTE: This is the Same file as "pluginlib/class_list_macros.h". It is copied
//       into surfr_core for compilation purposes. Thus, it provides No
//       functionality without the pluginlib ROS package.


namespace surfr {

  /*! \brief Base class for the Surgical Framework devices
   * 
   * This class provides a core set of tools which are common to all
   * protocol-related devices, such as follow up of the device status,
   * workflow definitions... The Base class is only intented to be used
   * as inherited from other protocol specialized classes.
   * 
   */
  class Base : public StateMachine {
  public:
  /*! \brief Constructor of the Base device class
   */
    Base();
  /*! \brief Destructor of the Base device class
   */
    virtual ~Base();
  /*! \brief Initialization of the Base device class
   * 
   *  \param cfg is a surfr::Params map with the following parameters:
   *         * \b simulation (required) -- A \a bool value to tell if this
   *           device works in simulation mode or with the real hardware.
   *         * \b frequency (required) -- A \a double value with the working
   *           frequency (refresh rate) of the device.
   *         * \b model (required) -- A \a std::string with the model tag to
   *           identify the device type (no spaces or special characters).
   *         * \b brand (optional) -- A \a std::string with the name of the
   *           manufacturer.
   *         * \b description (optional) -- A \a std::string with the
   *           description of the device and its purpose.
   *         * \b outputs/oN (optional) -- Each \a oN consists of a
   *           \a std::string with the type of that output.
   *
   *  \param id is an unsigned integer value with a unique identifier
   *         label for the object. By default this value will be 0.
   */
    void          init(const Params& cfg, uint32_t id = 0);
  /*! \brief Returns if the device works in simulation mode
   * 
   *  \return A bool value which tells if the device works in simulation
   *          mode.
   */
    bool          is_simulation();
  /*! \brief Returns the device identifier
   * 
   *  \return An unsigned int value which corresponds to the Communication
   *          identifier. If identifier was not set at object construction,
   *          then this function returns 0.
   */
    uint32_t      id();
  /*! \brief Returns the working frequency of the device
   */
    double_t      hz();
  /*! \brief Returns the time spent on each sample of the device
   */
    double_t      dt();
  /*! \brief Returns the brand of the device
   */
    std::string   brand();
  /*! \brief Returns the model identifier of the device
   */
    std::string   model();
  /*! \brief Returns the full name (model_id) of the device
   */
    std::string   name();
  /*! \brief Returns a brief description of the device
   */
    std::string   description();
  /*! \brief Returns the configuration parameters of the device
   */
    Params        config();

    virtual void output(Flow& data) = 0;
    virtual void input(Flow& data) = 0;
    virtual bool service(Flow& data) = 0;

  protected:
    bool          _is_simulation;                                               /*!< The device is running in simulation.       */
    uint32_t      _id;                                                          /*!< Identifier of the device object.           */
    double_t      _frequency;                                                   /*!< Sampling time (Hz) of the control loop.    */
    std::string   _brand;                                                       /*!< Description of the device manufacturer.    */
    std::string   _model;                                                       /*!< Description of the device model.           */
    std::string   _name;                                                        /*!< Tag with model+id (format: "Model_Id").    */
    std::string   _desc;                                                        /*!< Description of the role of this device.    */
    Params        _config;                                                      /*!< Map with all configuration parameters.     */
    Flow          _out;                                                         /*!< Data received from the device.             */
  };

} // namespace surfr


#endif  //_SURFR_CORE_BASE_H
