/*
 *
 *  surfr_core/comm/comm.h
 * 
 *               Communication definitions for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : July 5, 2017
 *  Revision   : July 5, 2017 (rev 1)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  Done List  : - Definition of all communication modes (R1)
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

#if !defined(_SURFR_CORE_COMM_H)
#define _SURFR_CORE_COMM_H

#include "surfr_core/core.h"


namespace surfr { namespace comm {
  using surfr::operator<<;

  /// Definition of variable types
  typedef enum { NONE = 0,
                 RECV = 1,
                 SEND = 2,
                 DUAL = 3
  } Mode;

  /// Constant values
  const std::map<Mode, std::string> MODE_DESC = { {Mode::NONE, "NONE"},
                                                  {Mode::RECV, "RECV"},
                                                  {Mode::SEND, "SEND"},
                                                  {Mode::DUAL, "DUAL"} };

} // namespace comm
} // namespace surfr

#endif  //_SURFR_CORE_COMM_H
