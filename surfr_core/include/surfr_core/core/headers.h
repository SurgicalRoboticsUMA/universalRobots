/*
 *
 *  surfr_core/core/headers.h
 * 
 *               Global headers for Surgical Framework
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


#if !defined(_SURFR_CORE_HEADERS_H)
#define _SURFR_CORE_HEADERS_H


#include <errno.h>                                // Standard error codes
#include <iostream>                               // Standard C++ I/O stream (cin, cout)
#include <iomanip>                                // Standard C++ I/O stream manipulation (hex, setw...)
#include <sstream>                                // Standard C++ string stream
#include <boost/bind.hpp>                         // Boost library bind method
#include <boost/lexical_cast.hpp>                 // Boost library data type conversion (value to string)
#include <boost/algorithm/string.hpp>             // Boost library string
#include <boost/algorithm/string/predicate.hpp>   // Boost library string comparison non-case sensitive

// #include "surfr_core/datatypes.h"                 // Surgical Frameworks flexible Data Types
//***** NOTE: THESE HEADERS WILL BE INCLUDED INTO DATATYPES.H! *****
#include <typeinfo>                               // Standard C++ library for typeid operator
#include <map>                                    // Standard C++ map class
#include <string>                                 // Standard C++ string class
#include <locale>                                 // Strings manipulation (std::toupper...)
#include <XmlRpcValue.h>                          // Generic value type of device params
#include <boost/any.hpp>                          // Generic type of a parameter
#include <Eigen/Eigen>                            // Maths with vectors and matrices
//***** NOTE: THESE HEADERS WILL BE INCLUDED INTO DATATYPES.H! *****


#endif  //_SURFR_CORE_HEADERS_H
