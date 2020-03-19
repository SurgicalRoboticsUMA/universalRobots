/*
 *
 *  surfr_core/core/templates.h
 * 
 *               Global templates for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : December 18, 2017
 *  Revision   : December 18, 2017 (rev 1)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
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


#if !defined(_SURFR_CORE_TEMPLATES_H)
#define _SURFR_CORE_TEMPLATES_H


#include "surfr_core/core/headers.h"              // Surgical Framework core headers


namespace surfr {
  /// Allows the use of << global overloads into ::surfr namespace
  using ::operator<<;

  /*! \brief Parse words of a string separated by spaces.
   *
   *  \param str is the string to be parsed.
   *  \sa parse_yaml()
   */
  template <class T>
  std::vector<T> parse_string(const std::string str) {
    T                 val;
    std::vector<T>    ret;
    std::stringstream ss(str);
    while (ss >> val) {
      ret.push_back(val);
    }
    return ret;
  };
  

  /*! \brief Conversion from a XmlRpcValue array to a vector array.
   *
   *  VERY IMPORTANT: This function assumes that the type T of each element
   *                  inside the vector array is the same as the values
   *                  stores into the XmlRpcValue array. If they are not
   *                  compatible the function will fail.
   *  \param x is the original XmlRpcValue array .
   *  \param v is the vector variable which stores the converted values.
   *  \sa vector2XmlRpcArray()
   */
  template <class T>
  void XmlRpcArray2vector(XmlRpc::XmlRpcValue& x, std::vector<T>& v) {
    // Check that the XmlRpcValue is an array
    //~ if (x.getType() == XmlRpc::XmlRpcValue::Type::TypeArray) {
    if (x.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      v.clear();
      for (unsigned int k = 0; k < x.size(); k++) {
        v.push_back(T(x[k]));
      }
    }
  }

  /*! \brief Conversion from a vector array to a XmlRpcValue array.
   *
   *  VERY IMPORTANT: This function assumes that the type T of each element
   *                  inside the vector array is the same as the values
   *                  stored into the XmlRpcValue array. If they are not
   *                  compatible the function will fail.
   *  \param v is the original vector array .
   *  \param x is the XmlRpcValue variable which stores the converted values.
   *  \sa XmlRpcArray2vector()
   */
  template <class T>
  void vector2XmlRpcArray(std::vector<T>& v, XmlRpc::XmlRpcValue& x) {
    // Check that the vector value is not empty
    if (!v.empty()) {
      x.clear();
      x.setSize(v.size());
      for (unsigned int k = 0; k < v.size(); k++) {
        x[k] = v[k];
      }
    }
  }

} // namespace surfr


/// Overload of << operator to allow std::vector<T> templated-type
template <class T>
std::ostream& operator<<(std::ostream &stream, const std::vector<T>& v) {
  stream << "[";
  for (int it = 0; it < v.size(); it++) {
    stream << v[it];
    if (it < v.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  return stream;
}


#endif  //_SURFR_CORE_TEMPLATES_H
