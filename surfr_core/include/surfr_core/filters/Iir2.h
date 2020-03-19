/*
 *
 *  surfr_core/filters/Iir2.h
 * 
 *               Order 2 Infinite Impulse Response (IIR) Filter class
 *               ----------------------------------------------------------
 *  Begin Date : November 10, 2017
 *  Revision   : November 10, 2017 (rev 1)
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

#if !defined(_SURFR_FILTER_IIR2_H)
#define _SURFR_FILTER_IIR2_H


#include "surfr_core/filters/FilterBase.h"


namespace surfr { namespace filters {
  /*! \brief Order 2 IIR filter class.
   * 
   * An IIR filter is a type of digital filter that has a feedback topology. The
   * order represents the number of input/previous output used to generate the
   * current output value.
   * 
   * The difference equation that defines this second order IIR filter is:
   * 
   * y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1] - a2*y[n-2]
   * 
   * Where b0, b1, a1, a2 are the filter coefficients.
   * 
   */
  class Iir2 : public FilterBase {
  public:
    /*! \brief IIR2 constructor.
     * 
     * This constructor can only be casted by passing the filter coefficients.
     * 
     * \param b0 is the first feedforward coefficient
     * \param b1 is the second feedforward coefficient
     * \param a1 is the first feedback coefficient
     * \param a2 is the second feedback coefficient
     */
    Iir2(XmlRpc::XmlRpcValue params);
    /*! \brief Reset the filter.
     * 
     * Reset the filter, so next call to evaluate method will be considered as the
     * first input value. All the previous one will be discarded.
     */
    void reset();
    /*! \brief Evaluate the filter for the new input.
     * 
     * \param value is the new input.
     */
    Eigen::VectorXd process(const Eigen::VectorXd& value);

  private:
    Eigen::VectorXd _x1;                                                        /*!< Previous input value.            */
    Eigen::VectorXd _y1;                                                        /*!< Previous output values.          */
    double _a1, _a2;                                                            /*!< Feedback filter coefficients.    */
    double _b0, _b1;                                                            /*!< Feedforward filter coefficients. */
    int    _order;                                                              /*!< Filter order, fills first "order-1"
                                                                                     samples with the original value. */
  };

} // namespace filters
} // namespace surfr

#endif  //_SURFR_FILTER_IIR2_H