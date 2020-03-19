/*
 *
 *  surfr_core/planners/Order1Linear.h
 * 
 *               Planner class for a First Order linear velocity trajectory
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

#include "surfr_core/planners/Order1Linear.h"


namespace surfr { namespace planners {

Order1Linear::Order1Linear(XmlRpc::XmlRpcValue params):
  PlannerBase(params),
  _dt((double_t) params["dt"]),
  _et(std::exp(-_dt/((double_t) params["tau"]))),
  _dx_max((double_t) params["max_velocity"]),
  _Kc((double_t) params["gain_control"]) {
  SURFR_DEBUG("Parameters of the Order1Linear planner:");
  SURFR_DEBUG("- Sampling time (dt)       : " + std::to_string(_dt));
  SURFR_DEBUG("- Constant e^(-dt/tau)     : " + std::to_string(_et));
  SURFR_DEBUG("- Maximum velocity (dx_max): " + std::to_string(_dx_max));
  SURFR_DEBUG("- Gain controller (Kc)     : " + std::to_string(_Kc));
  SURFR_DEBUG("Done");
}

Order1Linear::~Order1Linear() {
}
 
Eigen::VectorXd Order1Linear::process(const Eigen::VectorXd& target,
                                      const Eigen::VectorXd& actual,
                                      const Eigen::VectorXd& velocity) {
// Linear trajectory to target
  Eigen::VectorXd X = target - actual;
  double_t        x = X.norm();
// Planned Velocity
  // Next maximum velocity vector (affected by the max. acceleration)
  double_t        dx = std::min(x*_et*(1 - _et)/_dt,
                                _et*velocity.norm() + (1 - _et)*_dx_max);
  // Velocity Gain Controller
  Eigen::VectorXd dX = Eigen::VectorXd(_Kc*dx*X/x);
  // NOTE: Gain values higher than 0.25 (tested up to 0.75) keeps the system
  //       stable, however overdamping effect may appear

  // Control of position and velocity
  return dX;
}

} }