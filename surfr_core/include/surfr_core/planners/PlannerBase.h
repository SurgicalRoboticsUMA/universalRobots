/*
 *
 *  surfr_core/planners/PlannerBase.h
 * 
 *               Generic Planner base class for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : December 1, 2017
 *  Revision   : December 1, 2017 (rev 1)
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


#if !defined(_SURFR_PLANNERBASE_H)
#define _SURFR_PLANNERBASE_H


#include "surfr_core/core/headers.h"              // Surgical Framework core Headers
#include "surfr_core/core/macros.h"               // Surgical Framework core Macros


namespace surfr { namespace planners {
  /*! \brief Generic Planner base class.
   * 
   */
  class PlannerBase {
  public:
    /*! \brief Planner base class constructor.
     * 
     */
    PlannerBase(XmlRpc::XmlRpcValue params) {};
    ~PlannerBase() {};
    /*! \brief Evaluate the planner for the new input.
     * 
     * \param value is the new input.
     */
    virtual Eigen::VectorXd process(const Eigen::VectorXd& target,
                                    const Eigen::VectorXd& actual,
                                    const Eigen::VectorXd& velocity) = 0;
  };
} // namespace planners
} // namespace surfr

#endif  //_SURFR_PLANNERBASE_H
