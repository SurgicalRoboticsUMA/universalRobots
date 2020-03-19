/*
 *
 *  surfr_core/filters/Filter.h
 * 
 *               Compilation of all Surgical Framework filters
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

#if !defined(_SURFR_FILTER_H)
#define _SURFR_FILTER_H


#include "surfr_core/core.h"
#include "surfr_core/filters/FilterBase.h"
#include "surfr_core/filters/Iir2.h"


namespace surfr { namespace filters {
  /// Possible filters available
  typedef enum { FILTER_NONE = 0,
                 FILTER_IIR2
  } Types;

  /// Constant values (NOTE: use at() method for map access instead of [])
  const std::map<Types, std::string> TYPES_DESC = { {FILTER_NONE, "None"},
                                                    {FILTER_IIR2, "Iir2"} };

  /*! \brief Generic Filter base class.
   * 
   */
  class Filter {
  public:
    /*! \brief Filter base class constructor.
     * 
     */
    Filter(const surfr::Params& cfg);
    ~Filter();
    const std::string&    name();
    void                  reset();
    Eigen::VectorXd       process(const Eigen::VectorXd& value);
    double_t              process(const double_t& value);
    std::vector<double_t> process(std::vector<double_t>& value);

  private:
    FilterBase*         _handle;
    XmlRpc::XmlRpcValue _params;
    Types               _type;
    std::string         _desc;
  };

} // namespace filters
} // namespace surfr

#endif  //_SURFR_FILTER_H
