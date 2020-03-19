/*
 *
 *  surfr_core/planners/Planner.h
 * 
 *               Compilation of all Surgical Framework filters
 *               ----------------------------------------------------------
 *  Begin Date : December 19, 2017
 *  Revision   : December 19, 2017 (rev 2)
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

#include "surfr_core/planners/Planner.h"


namespace surfr { namespace planners {

Planner::Planner(const surfr::Params& cfg):
  _type(PLANNER_NONE),
  _desc(cfg.begin()->first),
  _params(cfg.begin()->second) {
  const std::map<Types, std::string>::const_iterator it = std::find_if(
    TYPES_DESC.begin(),
    TYPES_DESC.end(),
    boost::bind(&std::map<Types, std::string>::value_type::second, _1) == _desc
  );
  _type = it->first;
  // std::cout << "Planner is " << _desc << ": pair values [" << it->first << ", " << it->second << "]" << std::endl;
  SURFR_DEBUG("Creating surfr::Planner of type " << _desc << " with parameters: " << _params);
  switch (_type) {
  case PLANNER_ORDER1LINEAR:
    _handle = new Order1Linear(_params);
  break;
  case PLANNER_ORDER1ANGULAR:
    _handle = new Order1Angular(_params);
  break;
  case PLANNER_ORDER1POSE:
    _handle = new Order1Pose(_params);
  break;
  }
  SURFR_DEBUG("Done");
}

Planner::~Planner() {
  delete _handle;
}

const std::string& Planner::name() {
  return _desc;
}

Eigen::VectorXd Planner::process(const Eigen::VectorXd& target,
                                 const Eigen::VectorXd& actual,
                                 const Eigen::VectorXd& velocity) {
  return _handle->process(target, actual, velocity);
}

// std::vector<double_t> Planner::process(const std::vector<double_t>& value) {
std::vector<double_t> Planner::process(const std::vector<double_t>& target,
                                       const std::vector<double_t>& actual,
                                       const std::vector<double_t>& velocity) {
  Eigen::Map<Eigen::VectorXd> conv_target((double *) target.data(), target.size()),
                  conv_actual((double *) actual.data(), actual.size()),
                  conv_velocity((double *) velocity.data(), velocity.size());
// std::cout << "Target  : " << conv_target.transpose() << std::endl;
// std::cout << "Actual  : " << conv_actual.transpose() << std::endl;
// std::cout << "Velocity: " << conv_velocity.transpose() << std::endl;
  Eigen::VectorXd plan = _handle->process(conv_target, conv_actual, conv_velocity);
  Eigen::Map<Eigen::VectorXd> conv_plan((double *) plan.data(), plan.size());
  return std::vector<double_t>(conv_plan.data(), conv_plan.data() + conv_plan.size());
}

} // namespace planners
} // namespace surfr
