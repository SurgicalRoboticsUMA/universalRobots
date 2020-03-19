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

#include "surfr_core/planners/Order1Angular.h"


namespace surfr { namespace planners {

Order1Angular::Order1Angular(XmlRpc::XmlRpcValue params):
  PlannerBase(params),
  _dt((double_t) params["dt"]),
  _et(std::exp(-_dt/((double_t) params["tau"]))),
  _dw_max((double_t) params["max_twist"]),
  _Kc((double_t) params["gain_control"]) {
  SURFR_DEBUG("Parameters of the Order1Angular planner:");
  SURFR_DEBUG("- Sampling time (dt)    : " + std::to_string(_dt));
  SURFR_DEBUG("- Constant e^(-dt/tau)  : " + std::to_string(_et));
  SURFR_DEBUG("- Maximum twist (dw_max): " + std::to_string(_dw_max));
  SURFR_DEBUG("- Gain controller (Kc)  : " + std::to_string(_Kc));
  SURFR_DEBUG("Done");
}

Order1Angular::~Order1Angular() {
}
 
Eigen::VectorXd Order1Angular::process(const Eigen::VectorXd& target,
                                       const Eigen::VectorXd& actual,
                                       const Eigen::VectorXd& velocity) {
// Angular trajectory to target
  Eigen::Vector3d A0(actual(0), actual(1), actual(2)),
                  Af(target(0), target(1), target(2)),
                  W0(velocity(0), velocity(1), velocity(2));
  Eigen::AngleAxisd W;
   W.fromRotationMatrix(Eigen::AngleAxisd(Af.norm(), Af/Af.norm()).toRotationMatrix()*
                        Eigen::AngleAxisd(A0.norm(), A0/A0.norm()).toRotationMatrix().transpose());
// Planned Velocity
  // Next maximum velocity vector (affected by the max. acceleration)
  double_t        dw = std::min(W.angle()*_et*(1 - _et)/_dt,
                                _et*W0.norm() + (1 - _et)*_dw_max);
  // Velocity Gain Controller
  Eigen::VectorXd dW = Eigen::VectorXd(_Kc*dw*W.axis());
  // NOTE: Gain values higher than 0.25 (tested up to 0.75) keeps the system
  //       stable, however overdamping effect may appear

  // Control of position and velocity
  return dW;
}

} }