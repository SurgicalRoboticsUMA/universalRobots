/*
 *
 *  surfr_core/StateMachine.cpp
 * 
 *               State Machine definition for Surgical Framework classes
 *               ----------------------------------------------------------
 *  Begin Date : June 12, 2017
 *  Revision   : June 20, 2017 (rev 3)
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

#include "surfr_core/StateMachine.h"


namespace surfr {

StateMachine::StateMachine() :
_state(STATE_LOADED) {
  STATE_DESC.insert( std::pair<State, std::string>(STATE_NONE      , "NONE"      ) );
  STATE_DESC.insert( std::pair<State, std::string>(STATE_LOADED    , "LOADED"    ) );
  STATE_DESC.insert( std::pair<State, std::string>(STATE_CONFIGURED, "CONFIGURED") );
  STATE_DESC.insert( std::pair<State, std::string>(STATE_STOPPED   , "STOPPED"   ) );
  STATE_DESC.insert( std::pair<State, std::string>(STATE_RUNNING   , "RUNNING"   ) );
  STATE_DESC.insert( std::pair<State, std::string>(STATE_EXCEPTION , "EXCEPTION" ) );
  STATE_DESC.insert( std::pair<State, std::string>(STATE_FATAL     , "FATAL"     ) );

  SURFR_DEBUG("Done");
}

StateMachine::~StateMachine() {
  SURFR_DEBUG("Done");
}

std::string StateMachine::state_desc() {
  return STATE_DESC.at(_state);
}

std::string StateMachine::state_desc(State s) {
  return STATE_DESC.at(s);
}

bool StateMachine::ok() {
  return ((_state == STATE_CONFIGURED) |
          (_state == STATE_STOPPED) |
          (_state == STATE_RUNNING));
}

bool StateMachine::configure() {
  // bool ret = (_state == STATE_LOADED) | (_state == STATE_STOPPED);
  bool ret = (_state == STATE_LOADED);
  if (!ret) {
      SURFR_DEBUG("This trigger cannot be called on state " + STATE_DESC.at(_state));
    } else {
      ret &= configureHook();
      if (ret) {
        _state = STATE_CONFIGURED;
      }
    }
    return ret;
}

bool StateMachine::cleanup() {
  bool ret = (_state == STATE_CONFIGURED);
  if (!ret) {
    SURFR_DEBUG("This trigger cannot be called on state " + STATE_DESC.at(_state));
  } else {
    ret &= cleanupHook();
    if (ret) {
      _state = STATE_CONFIGURED;
    }
  }
  return ret;
}

bool StateMachine::start() {
  bool ret = (_state == STATE_CONFIGURED) |
             (_state == STATE_STOPPED);
  if (!ret) {
    SURFR_DEBUG("This trigger cannot be called on state " + STATE_DESC.at(_state));
  } else {
    ret &= startHook();
    if (ret) {
      _state = STATE_RUNNING;
    }
  }
  return ret;
}

bool StateMachine::stop() {
  bool ret = (_state == STATE_RUNNING);
  if (!ret) {
    SURFR_DEBUG("This trigger cannot be called on state " + STATE_DESC.at(_state));
  } else {
    ret &= stopHook();
    if (ret) {
      _state = STATE_STOPPED;
    }
  }
  return ret;
}

bool StateMachine::update() {
  bool ret = (_state == STATE_RUNNING);
  if (!ret) {
    SURFR_DEBUG("This trigger cannot be called on state " + STATE_DESC.at(_state));
  } else {
    ret &= updateHook();
  }
  return ret;
};

//!TODO BEGIN
bool StateMachine::exception() {
  return false;
}

bool StateMachine::fatal() {
  return false;
}
//!TODO END

bool StateMachine::is_loaded()     { return (_state == STATE_LOADED);     }
bool StateMachine::is_configured() { return (_state == STATE_CONFIGURED); }
bool StateMachine::is_stopped()    { return (_state == STATE_STOPPED);    }
bool StateMachine::is_running()    { return (_state == STATE_RUNNING);    }

bool StateMachine::configureHook() { return true; }
bool StateMachine::cleanupHook()   { return true; }
bool StateMachine::startHook()     { return true; }
bool StateMachine::stopHook()      { return true; }
bool StateMachine::updateHook()    { return true; }
bool StateMachine::exceptionHook() { return true; }

} // namespace surfr
