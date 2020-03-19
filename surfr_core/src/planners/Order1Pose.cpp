/*
 *
 *  surfr_core/planners/Order1Pose.h
 * 
 *               Planner class for a First Order pose velocity trajectory
 *               ----------------------------------------------------------
 *  Begin Date : December 19, 2017
 *  Revision   : December 20, 2017 (rev 2)
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

#include "surfr_core/planners/Order1Pose.h"


namespace surfr { namespace planners {

Order1Pose::Order1Pose(XmlRpc::XmlRpcValue params):
  PlannerBase(params),
  _dt((double_t) params["dt"]),
  _et(std::exp(-_dt/((double_t) params["tau"]))),
  _dx_max((double_t) params["max_velocity"]),
  _dw_max((double_t) params["max_twist"]),
  _Kc((double_t) params["gain_control"]) {
  SURFR_DEBUG("Parameters of the Order1Pose planner:");
  SURFR_DEBUG("- Sampling time (dt)       : " + std::to_string(_dt));
  SURFR_DEBUG("- Constant e^(-dt/tau)     : " + std::to_string(_et));
  SURFR_DEBUG("- Maximum velocity (dx_max): " + std::to_string(_dx_max));
  SURFR_DEBUG("- Maximum twist (dw_max)   : " + std::to_string(_dw_max));
  SURFR_DEBUG("- Gain controller (Kc)     : " + std::to_string(_Kc));
  SURFR_DEBUG("Done");
}

Order1Pose::~Order1Pose() {
}
 
Eigen::VectorXd Order1Pose::process(const Eigen::VectorXd& target,
                                    const Eigen::VectorXd& actual,
                                    const Eigen::VectorXd& velocity) {
// Linear trajectory to target
  Eigen::Vector3d P = Eigen::Vector3d(target(0), target(1), target(2)) -
                      Eigen::Vector3d(actual(0), actual(1), actual(2)),
                  V0(velocity(0), velocity(1), velocity(2));
  double_t        p = P.norm();
// Angular trajectory to target
  Eigen::Vector3d A0(actual(3), actual(4), actual(5)),
                  Af(target(3), target(4), target(5)),
                  W0(velocity(3), velocity(4), velocity(5));
  Eigen::AngleAxisd W;
   W.fromRotationMatrix(Eigen::AngleAxisd(Af.norm(), Af/Af.norm()).toRotationMatrix()*
                        Eigen::AngleAxisd(A0.norm(), A0/A0.norm()).toRotationMatrix().transpose());
// Planned Velocity
  // Next maximum velocity vector (affected by the max. acceleration)
  double_t        dp = std::min(p*_et*(1 - _et)/_dt,
                                _et*V0.norm() + (1 - _et)*_dx_max);
  double_t        dw = std::min(W.angle()*_et*(1 - _et)/_dt,
                                _et*W0.norm() + (1 - _et)*_dw_max);
  // Velocity Gain Controller
  Eigen::Vector3d dP = Eigen::Vector3d(_Kc*dp*P/p);
  Eigen::Vector3d dW = Eigen::Vector3d(_Kc*dw*W.axis());
  Eigen::VectorXd ret(6);
    ret << dP(0), dP(1), dP(2), dW(0), dW(1), dW(2);
  // NOTE: Gain values higher than 0.25 (tested up to 0.75) keeps the system
  //       stable, however overdamping effect may appear

  // Control of position and velocity
  return ret;
}

} }