/*
 *
 *  surfr_core/StateMachine.h
 * 
 *               State Machine definition for Surgical Framework classes
 *               ----------------------------------------------------------
 *  Begin Date : June 12, 2017
 *  Revision   : June 20, 2017 (rev 3)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Implementation of exception() method.
 *               - Implementation of fatal() method.
 *               - Use of try-catch-throw for exceptions handling.
 *               - Full review of Doxygen documentation.
 *  Done List  : - Define all possible states (R1)
 *               - Customization of triggers by Hook() methods (R1)
 *               - Addition of comments (R2)
 *               - Move code to cpp file (R3)
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

#if !defined(_SURFR_STATEMACHINE_H)
#define _SURFR_STATEMACHINE_H


#include "surfr_core/core/headers.h"              // Surgical Framework core Headers
#include "surfr_core/core/macros.h"               // Surgical Framework core Macros


namespace surfr {
  /// Allows the use of << global overloads into ::surfr namespace
  using ::operator<<;

class StateMachine {
  public:
  /// Possible states of a device
    typedef enum { STATE_NONE = 0,
                   STATE_LOADED,
                   STATE_CONFIGURED,
                   STATE_STOPPED,
                   STATE_RUNNING,
                   STATE_EXCEPTION,
                   STATE_FATAL
                  } State;

  /// Constant values (NOTE: use at() method for map access instead of [])
  //~ const std::map<State, std::string> STATE_DESC = { {STATE_NONE      , "NONE"      },
                                                    //~ {STATE_LOADED    , "LOADED"    },
                                                    //~ {STATE_CONFIGURED, "CONFIGURED"},
                                                    //~ {STATE_STOPPED   , "STOPPED"   },
                                                    //~ {STATE_RUNNING   , "RUNNING"   },
                                                    //~ {STATE_EXCEPTION , "EXCEPTION" },
                                                    //~ {STATE_FATAL     , "FATAL"     } };
    std::map<State, std::string> STATE_DESC;

    /*! \brief Constructor of the State Machine class
     */
    StateMachine();
    /*! \brief Destructor of the State Machine class
     */
    ~StateMachine();
    std::string state_desc();
    std::string state_desc(State s);
    /*! \brief Returns if the State Machine is at its normal workflow. This
     *         happens when the current state is "Loaded", "Stopped" or
     *         "Running".
     * 
     *  \return A bool value which is \a true if state is one of the previous
     *          values, \a false otherwise.
     */
    bool ok();
    /*! \brief Attempts to trigger the state machine configuration method,
     *         which can be run from "Loaded" or "Stopped" states. The
     *         configuration data are received by the \a cfg parameter of type
     *         surfr::Param. If the configuration can be updated then the state
     *         is set to "Stopped".
     * 
     *  \param cfg is a surfr::Param map duple, where the first term is a
     *         std::string with the description of the configuration parameter,
     *         and the second term is a XmlRpc::XmlRpcValue with the contents
     *         of such parameter.
     *  \return A bool value which is \a true on configuration success, \a false
     *          otherwise.
     *  \sa cleanup(), start(), stop()
     */
    // bool configure(const Params& cfg);
    bool configure();
    /*! \brief Attempts to trigger the state machine cleanup method, which can
     *         be run from "Stopped" state. If the cleanup could be applied
     *         then the state is set to "Loaded".
     * 
     *  \return A bool value which is \a true on cleanup success, \a false
     *          otherwise.
     *  \sa configure(), start(), stop()
     */
    bool cleanup();
    /*! \brief Triggers the State Machine from "Stopped" to "Running". This
     *         method has no effect if called from other state different of
     *         "Stopped". Besides, the trigger function calls a virtual method
     *         called \a startHook() that can be customized by the child class,
     *         which inherits \a StateMachine. The trigger will be applied only
     *         if \a startHook() returns \a true too.
     * 
     *  \return A bool value with \a true value if the trigger has been enabled,
     *          \false otherwise.
     *  \sa configure(), cleanup(), stop()
     */
    bool start();
    /*! \brief Triggers the State Machine from "Running" to "Stopped". This
     *         method has no effect if called from other state different of
     *         "Running". Besides, the trigger function calls a virtual method
     *         called \a stopHook() that can be customized by the child class,
     *         which inherits \a StateMachine. The trigger will be applied only
     *         if \a stopHook() returns \a true too.
     * 
     *  \return A bool value with \a true value if the trigger has been enabled,
     *          \false otherwise.
     *  \sa configure(), cleanup(), start()
     */
    bool stop();
    bool update();
    bool exception();
    bool fatal();

    /*! \brief Check if the State Machine is at "Loaded" state.
     * 
     *  \return A bool value with \a true value if the state is "Loaded",
     *          \false otherwise.
     *  \sa is_stopped(), is_running()
     */
    bool is_loaded();
    bool is_configured();
    /*! \brief Check if the State Machine is at "Stopped" state.
     * 
     *  \return A bool value with \a true value if the state is "Stopped",
     *          \false otherwise.
     *  \sa is_loaded(), is_running()
     */
    bool is_stopped();
    /*! \brief Check if the State Machine is at "Running" state.
     * 
     *  \return A bool value with \a true value if the state is "Running",
     *          \false otherwise.
     *  \sa is_loaded(), is_stopped()
     */
    bool is_running();

  protected:
    // virtual bool configureHook(const Params& cfg);
    virtual bool configureHook();
    virtual bool cleanupHook();
    virtual bool startHook();
    virtual bool stopHook();
    virtual bool updateHook();
    virtual bool exceptionHook();

    State        _state;                          /*!< State of the device.                       */
  };

} // namespace surfr

#endif  //_SURFR_STATEMACHINE_H
