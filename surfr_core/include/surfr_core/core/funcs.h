/*
 *
 *  surfr_core/core/funcs.h
 * 
 *               Global functions for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : December 18, 2017
 *  Revision   : March 14, 2018 (rev 2)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  Done List  : - Added a new function to decode a Params into XmlRpc
 *                 value. Actually only works for 1 level of the XML
 *                 tree (R2)
 *               - Initial release, splitted from core.h (R1)
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


#if !defined(_SURFR_CORE_FUNCS_H)
#define _SURFR_CORE_FUNCS_H


#include "surfr_core/core/headers.h"              // Surgical Framework core headers
#include "surfr_core/core/types.h"                // Surgical Framework core Type Definitions
#include "surfr_core/core/macros.h"               // Surgical Framework core Macros


namespace surfr {
  /// Allows the use of << global overloads into ::surfr namespace
  using ::operator<<;

  /// Global values
  // bool    IS_INIT  = false;
  // bool    IS_DEBUG = false;


  /*! \brief Starts the Surgical Framework system
   *
   * List of valid arguments:
   * --debug   Enables the debug mode (extra information on terminal)
   * --nodebug Disables the debug mode (default option)
   *
   * \param argc is the number of arguments provided by main() function.
   * \param argv is the array of strings with each argument provided by the main() function.
   */
  int init(int argc, char **argv);
  int init(int argc, std::vector<std::string> argv);

  bool is_debug();
  bool is_init();

  // Return all parameters within path namespace of a yaml file data into list variable 
  /*! \brief Parse parameters from a yaml file data.
   *
   *  Return all parameters within a specific path namespace of a yaml file data, which is
   *  stored into a map of XmlRpc::XmlRpcValue indexed by std::string.
   *
   *  \param list is the map where the parsed data is returned.
   *  \param yaml is where all the yaml data file is passed.
   *  \param path is tha path namespace where the desired data to be parsed is stored.
   *  \param model is for internal use (leave empty).
   *  \sa parse_string()
   */
  void parse_yaml(Params& list, XmlRpc::XmlRpcValue& yaml, const std::string& path, const std::string& model);

  XmlRpc::XmlRpcValue unparse_params(Params& list, const std::string& root = "");

  /*! \brief Conversion from a 2-byte array to an integer value.
   *
   *  \param b is an array string with the integer value in ascii code.
   *  \sa bytes2double()
   */
  int bytes2int(char * b);
  /*! \brief Conversion from a 4-byte array to a double value.
   *
   *  \param b is an array string with the double value in ascii code.
   *  \sa bytes2int()
   */
  double bytes2double(char * b);

Params::const_iterator findKey(const Params& map, const std::string& search_for);

  /// Overload of << operator to allow Params type
  std::ostream& operator<<(std::ostream &stream, const Params& p);

} // namespace surfr


#endif  //_SURFR_CORE_FUNCS_H
