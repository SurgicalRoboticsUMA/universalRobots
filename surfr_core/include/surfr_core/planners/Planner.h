/*
 *
 *  surfr_core/planners/Planner.h
 * 
 *               Compilation of all Surgical Framework filters
 *               ----------------------------------------------------------
 *  Begin Date : December 19, 2017
 *  Revision   : December 20, 2017 (rev 3)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : 
 *  Done List  : - Split the Order1 planner into three versions: linear,
 *                 angular and mixed-posed trajectories (R2)
 *               - Separation of the class code into a cpp file (R2)
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


#if !defined(_SURFR_PLANNER_H)
#define _SURFR_PLANNER_H


#include "surfr_core/core.h"
#include "surfr_core/planners/PlannerBase.h"
#include "surfr_core/planners/Order1Linear.h"
#include "surfr_core/planners/Order1Angular.h"
#include "surfr_core/planners/Order1Pose.h"


namespace surfr { namespace planners {
  /// Possible planners available
  typedef enum { PLANNER_NONE = 0,
                 PLANNER_ORDER1LINEAR,
                 PLANNER_ORDER1ANGULAR,
                 PLANNER_ORDER1POSE
  } Types;

  /// Constant values (NOTE: use at() method for map access instead of [])
  const std::map<Types, std::string> TYPES_DESC = { {PLANNER_NONE         , "None"},
                                                    {PLANNER_ORDER1LINEAR , "Order1Linear"},
                                                    {PLANNER_ORDER1ANGULAR, "Order1Angular"},
                                                    {PLANNER_ORDER1POSE   , "Order1Pose"} };

  /*! \brief Main Planner class.
   * 
   */
  class Planner {
  public:
    /*! \brief Main Planner class constructor.
     * 
     */
    Planner(const surfr::Params& cfg);
    ~Planner();
    const std::string& name();
    Eigen::VectorXd       process(const Eigen::VectorXd& target,
                                  const Eigen::VectorXd& actual,
                                  const Eigen::VectorXd& velocity);
    std::vector<double_t> process(const std::vector<double_t>& target,
                                  const std::vector<double_t>& actual,
                                  const std::vector<double_t>& velocity);

  private:
    PlannerBase*        _handle;
    XmlRpc::XmlRpcValue _params;
    Types               _type;
    std::string         _desc;
  };

} // namespace planners
} // namespace surfr

#endif  //_SURFR_PLANNER_H
