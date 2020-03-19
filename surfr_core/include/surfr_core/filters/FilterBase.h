/*
 *
 *  surfr_core/filters/FilterBase.h
 * 
 *               Generic Filter base class for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : November 15, 2017
 *  Revision   : December 12, 2017 (rev 2)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : 
 *  Done List  : - Split the source code from the header data (R2)
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

#if !defined(_SURFR_FILTERBASE_H)
#define _SURFR_FILTERBASE_H


#include "surfr_core/core/headers.h"              // Surgical Framework core Headers
#include "surfr_core/core/macros.h"               // Surgical Framework core Macros


namespace surfr { namespace filters {

  /*! \brief Generic Filter base class.
   * 
   */
  class FilterBase {
  public:
    /*! \brief Filter base class constructor.
     * 
     * This constructor can only be casted by passing the filter coefficients.
     * 
     * \param b0 is the first feedforward coefficient
     * \param b1 is the second feedforward coefficient
     * \param a1 is the first feedback coefficient
     * \param a2 is the second feedback coefficient
     */
    FilterBase(XmlRpc::XmlRpcValue params) {};
    ~FilterBase() {};
    /*! \brief Reset the filter.
     * 
     * Reset the filter, so next call to evaluate method will be considered as the
     * first input value. All the previous one will be discarded.
     */
    virtual void reset() = 0;
    /*! \brief Evaluate the filter for the new input.
     * 
     * \param value is the new input.
     */
    virtual Eigen::VectorXd process(const Eigen::VectorXd& value) = 0;
  };

} // namespace filters
} // namespace surfr


#endif  //_SURFR_FILTERBASE_H
