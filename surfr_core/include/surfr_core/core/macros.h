/*
 *
 *  surfr_core/core/macros.h
 * 
 *               Global macros for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : December 18, 2017
 *  Revision   : October 26, 2018 (rev 2)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  Done List  : - Changed std::to_string() to boost::lexical_cast<>().
 *                 This improves compatibility with previous versions of
 *                 ROS that are unable to compile with the C++11
 *                 standard (R2)
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


#if !defined(_SURFR_CORE_MACROS_H)
#define _SURFR_CORE_MACROS_H

#include "surfr_core/core/headers.h"              // Surgical Framework core headers
#include "surfr_core/core/funcs.h"                // Surgical Framework core functions


#define DEFAULT_FREQUENCY 25


/// Macros
#define SURFR_DEBUG(msg)             surfr::is_debug() ? (std::cout << "\033[32m[DEBUG]["     << std::string(__FILE__).substr(std::string(__FILE__).rfind("/") + 1) << "::" << boost::lexical_cast<std::string>(__LINE__) << "::" << std::string(__FUNCTION__) << "()] " << msg << "\033[0m\n") : (std::cout << "")
#define SURFR_INFO(msg)                                   std::cout << "\033[37;1m[INFO]["    << std::string(__FILE__).substr(std::string(__FILE__).rfind("/") + 1) << "::" << boost::lexical_cast<std::string>(__LINE__) << "::" << std::string(__FUNCTION__) << "()] " << msg << "\033[0m\n"
#define SURFR_WARNING(msg)                                std::cout << "\033[33;1m[WARNING][" << std::string(__FILE__).substr(std::string(__FILE__).rfind("/") + 1) << "::" << boost::lexical_cast<std::string>(__LINE__) << "::" << std::string(__FUNCTION__) << "()] " << msg << "\033[0m\n"
#define SURFR_ERROR(msg)                                  std::cout << "\033[31;1m[ERROR]["   << std::string(__FILE__).substr(std::string(__FILE__).rfind("/") + 1) << "::" << boost::lexical_cast<std::string>(__LINE__) << "::" << std::string(__FUNCTION__) << "()] " << msg << "\033[0m\n"
#define FLOW_CAST(data, type, param) boost::any_cast<type>((data)[param])


#endif  //_SURFR_CORE_MACROS_H
