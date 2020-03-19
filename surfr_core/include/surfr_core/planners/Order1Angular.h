/*
 *
 *  surfr_core/planners/Order1Angular.h
 * 
 *               Planner class for a First Order angular velocity trajectory
 *               ----------------------------------------------------------
 *  Begin Date : December 19, 2017
 *  Revision   : December 20, 2017 (rev 2)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Remove the control part (gain parameter) and move it to
 *                 another library for controllers. As an alternative, use
 *                 a current ROS package for controllers.
 *  Done List  : - Split the Order1 planner into three versions: linear,
 *                 angular and mixed-posed trajectories (R2)
 *               - Initial release (R1)
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

#if !defined(_SURFR_PLANNER_ORDER1ANGULAR_H)
#define _SURFR_PLANNER_ORDER1ANGULAR_H


#include "surfr_core/planners/PlannerBase.h"


namespace surfr { namespace planners {
  /*! \brief Order 1 velocity planner class.
   * 
   * This planner generates a 3-step trajectory with the velocities:
   * - A 1st order profile in velocity for the acceleration
   * - A constant maximum velocity
   * - A 1st order profile in velocity for the deceleration
   *
   * The difference equations that defines this planner are:
   * 
   * dX[n] = Kv*(Xf - dX[n-1]*dt)*e^(-t/tau)*(1 - e^(-t/tau))/dt
   * dX[n] = Kv*e^(-t/tau)*dX[n-1] + (1 - e^(-t/tau))*Vmax
   * 
   */
  class Order1Angular : public PlannerBase {
  public:
    /*! \brief Order 1 Planner constructor.
     * 
     */
    Order1Angular(XmlRpc::XmlRpcValue params);
    ~Order1Angular();
    /*! \brief Evaluate the planner for the new input.
     * 
     * \param value is the new input.
     */
    Eigen::VectorXd process(const Eigen::VectorXd& target,
                            const Eigen::VectorXd& actual,
                            const Eigen::VectorXd& velocity);

  private:
    const double_t _dt;
    const double_t _et;
    const double_t _dw_max;
    const double_t _Kc;
  };

} // namespace planners
} // namespace surfr

#endif  //_SURFR_PLANNER_ORDER1ANGULAR_H