/*
 *
 *  surfr_core/core.h
 * 
 *               Global definitions for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : September 14, 2016
 *  Revision   : December 18, 2017 (rev 16)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  Done List  : - Additional split of the header file into headers list,
 *                 templates, functions and macros. Now the core.h header
 *                 only calls the splitted headers (R16)
 *               - Split the source code from the header file. This allows
 *                 the use of a unique library for the Core System (R15)
 *               - New function to parse a string into a vector (R14)
 *               - Message macros now are "streamed" (R13)
 *               - Multiple updates (R4-R12)
 *               - Move code to cpp file (R3)
 *               - Addition of comments (R2)
 *               - Customization of triggers by Hook() methods (R1)
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


#if !defined(_SURFR_CORE_H)
#define _SURFR_CORE_H


#include "surfr_core/core/headers.h"              // Surgical Framework core Headers
#include "surfr_core/core/macros.h"               // Surgical Framework core Macros
#include "surfr_core/core/types.h"                // Surgical Framework core Type Definitions
#include "surfr_core/core/funcs.h"                // Surgical Framework core Functions
#include "surfr_core/core/templates.h"            // Surgical Framework core Templates


#endif  //_SURFR_CORE_H
