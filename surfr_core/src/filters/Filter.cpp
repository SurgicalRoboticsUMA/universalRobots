/*
 *
 *  surfr_core/filters/Filter.cpp
 * 
 *               Compilation of all Surgical Framework filters
 *               ----------------------------------------------------------
 *  Begin Date : November 15, 2017
 *  Revision   : December 12, 2017 (rev 2)
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

#include "surfr_core/filters/Filter.h"


namespace surfr { namespace filters {

Filter::Filter(const surfr::Params& cfg):
  _type(FILTER_NONE),
  _desc(cfg.begin()->first),
  _params(cfg.begin()->second) {

  const std::map<Types, std::string>::const_iterator it = std::find_if(
    TYPES_DESC.begin(),
    TYPES_DESC.end(),
    boost::bind(&std::map<Types, std::string>::value_type::second, _1) == _desc
  );
  _type = it->first;
  // std::cout << "Filter is " << _desc << ": pair values [" << it->first << ", " << it->second << "]" << std::endl;
  SURFR_DEBUG("Creating surfr::Filter of type " << _desc << " with parameters: " << _params);
  switch (_type) {
  case FILTER_IIR2:
    _handle = new Iir2(_params);
  break;
  }
  SURFR_DEBUG("Done");
}

Filter::~Filter() {
  delete _handle;
}

const std::string& Filter::name() {
  return _desc;
}

void Filter::reset() {
  _handle->reset();
}

Eigen::VectorXd Filter::process(const Eigen::VectorXd& value) {
  return _handle->process(value);
}

double_t Filter::process(const double_t& value) {
  Eigen::VectorXd conv(1);
  conv << value;
  conv = _handle->process(conv);
  return conv(0);
}

std::vector<double_t> Filter::process(std::vector<double_t>& value) {
  Eigen::Map<Eigen::VectorXd> conv(value.data(), value.size());
  conv = _handle->process(conv);
  return std::vector<double_t>(conv.data(), conv.data() + conv.size());
}

} // namespace filters
} // namespace surfr
