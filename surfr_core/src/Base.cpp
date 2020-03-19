/*
 *
 *  surfr_core/Base.cpp
 * 
 *               Base class for Surgical Framework devices
 *               ----------------------------------------------------------
 *  Begin Date : November 14, 2016
 *  Revision   : March 23, 2018 (rev 8)
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

#include "surfr_core/Base.h"


namespace surfr {

Base::Base() :
StateMachine::StateMachine() {
  SURFR_DEBUG("Done");
}

void Base::init(const Params& cfg, uint32_t id) {
  _model  = "models/";
  _config = cfg;
  _id     = id;
    if (!_id) SURFR_WARNING("Identifier not set, using 0 by default.");

/*
 * Extraction of the configuration parameters
 */
  // SIMULATION
  if (_config.find("simulation") != _config.end()) {
    try {
      _is_simulation = bool(_config.at("simulation"));
      SURFR_DEBUG("Simulation: " << _is_simulation);
    } catch(std::exception const err) {
      SURFR_ERROR(err.what());
    }
  } else {
    _is_simulation = true;
    _config["simulation"] = XmlRpc::XmlRpcValue(true);
    SURFR_WARNING("Undefined simulation mode, set to default value (true).");
  }
  // FREQUENCY
  if (_config.find("frequency") != _config.end()) {
    try {
      _frequency = double(int(_config.at("frequency")));
      SURFR_DEBUG("Frequency: " << _frequency);
    } catch(std::exception const err) {
      SURFR_ERROR(err.what());
    }
  } else {
    _frequency = DEFAULT_FREQUENCY;
    SURFR_WARNING("Undefined frequency, set to default value (" << _frequency << ").");
  }
  // BRAND
  if (_config.find("brand") != _config.end()) {
    try {
      _brand = std::string(_config.at("brand"));
      SURFR_DEBUG("Brand: " << _brand);
    } catch(std::exception const err) {
      SURFR_ERROR(err.what());
    }
  } else {
    _brand = "";
    SURFR_DEBUG("Undefined brand.");
  }
  // MODEL, DESCRIPTION
  for (Params::const_iterator it = _config.begin(); it != _config.end(); it++) {
    if (it->first.find(_model) != std::string::npos) {
      const std::string& tag = it->first;
      _model  = tag.substr(_model.length(), tag.rfind('/') - _model.length());
      SURFR_DEBUG("Model: " << _model);
      try {
        _desc = std::string((XmlRpc::XmlRpcValue) it->second);
        SURFR_DEBUG("Description: " << _desc);
      } catch(std::exception const err) {
        SURFR_ERROR(err.what());
      }
      break;
    }
    if (std::next(it) == _config.end()) {
      SURFR_ERROR("Model not found");
      return;
    }
  }
  // OUTPUTS
  for (Params::iterator it = _config.begin(); it != _config.end(); it++) {
    if ((it->first).find("outputs/") == 0) {
      std::string param, type;
      try {
        param = (it->first).substr(8, (it->first).size() - 8);
        type  = std::string(it->second);
      } catch(std::exception const err) {
        SURFR_ERROR(err.what());
      }

      // All output parameters are defined in a device_outputs.yaml configuration
      // file that must be loaded on a .LAUNCH file before this node. Each
      // parameter sets a variable type among the available ones.
      if        (!type.compare("Boolean" )) {
        (_out)[param] = bool(false);
      } else if (!type.compare("Unsigned")) {
        (_out)[param] = (unsigned int)(0);
      } else if (!type.compare("Integer" )) {
        (_out)[param] = int(0);
      } else if (!type.compare("Double"  )) {
        (_out)[param] = double(0.0);
      } else if (!type.compare("String"  )) {
        (_out)[param] = std::string("");
      } else if (!type.compare("Vector" )) {
        (_out)[param] = std::vector<double>(1);
      } else if (!type.compare("Joint"   )) {
        (_out)[param + "/name"       ] = std::vector<std::string>(1);
        (_out)[param + "/state"      ] = std::vector<unsigned int>(1);
        (_out)[param + "/position"   ] = std::vector<double>(1);
        (_out)[param + "/velocity"   ] = std::vector<double>(1);
        (_out)[param + "/current"    ] = std::vector<double>(1);
        (_out)[param + "/effort"     ] = std::vector<double>(1);
        (_out)[param + "/temperature"] = std::vector<double>(1);
      } else {
        SURFR_DEBUG("Param '" << param << "' of type '" << type << "' is not valid, ignored.");
        continue;
      }
      SURFR_DEBUG("(_out)[" << param << "] = " << type);
    }
  }

  _state = STATE_LOADED;
  SURFR_DEBUG("Done");
}

Base::~Base() {
  SURFR_DEBUG("Done");
}

bool Base::is_simulation() {
  return _is_simulation;
}

uint32_t Base::id() {
  return _id;
}

double_t Base::hz() {
  return _frequency;
}

double_t Base::dt() {
  return 1.0/_frequency;
}

std::string Base::brand() {
  return _brand;
}

std::string Base::model() {
  return _model;
}

std::string Base::name() {
  return _model + "_" + std::to_string(_id);
}

std::string Base::description() {
  return _desc;
}

Params Base::config() {
  return _config;
}

} // namespace surfr
