/*
 *
 *  surfr_core/core/funcs.cpp
 * 
 *               Global definitions for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : December 18, 2017
 *  Revision   : March 14, 2018 (rev 2)
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


// #include <surfr_core/core.h>
#include "surfr_core/core/funcs.h"


namespace surfr {
/// Global values
bool IS_INIT  = false;
bool IS_DEBUG = false;


int init(int argc, char **argv) {
//   static bool IS_INIT  = false;
//   static bool IS_DEBUG = false;

  for (int it = 0; it < argc; it++) {
    if (!std::string(argv[it]).compare("--debug")) {
      IS_DEBUG = true;
    }
    if (!std::string(argv[it]).compare("--nodebug")) {
      IS_DEBUG = false;
    }
    // std::cout << "." << argv[it] << "." << std::endl;
  }
  IS_INIT = true;
}

int init(int argc, std::vector<std::string> argv) {
  std::vector<char*> cstr;
    cstr.reserve(argc);
    for(size_t k = 0; k < argc; k++)
      cstr.push_back(const_cast<char*>(argv[k].c_str()));

  return init(argc, &cstr[0]);
}

bool is_debug() {
    return IS_DEBUG;
}

bool is_init() {
    return IS_INIT;
}

void parse_yaml(Params& list, XmlRpc::XmlRpcValue& yaml, const std::string& path, const std::string& model) {
  if (yaml.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    for(surfr::Params::iterator it = yaml.begin(); it != yaml.end(); it++) {
      if (path.compare("/models") || !it->first.compare(model)) {
        parse_yaml(list, it->second, path + "/" + it->first, model);
      }
    }
  } else {
    list[path.substr(1)] = yaml;
    SURFR_DEBUG(path << ": " << yaml);
  }
}

// NOTE: Actually, this function only works for 1 level of the XML tree.
//       It must be updated to work recursively into sublevels.
XmlRpc::XmlRpcValue unparse_params(Params& list, const std::string& root) {
  std::string         label = root + (root.back() == '/' ? "" : "/");
  //                     xml = "<value><struct>";

  // for (Params::iterator it = list.begin(); it != list.end(); it++) {
  //   if ((root == "") || (it->first.find(root) != std::string::npos)) {
  //     label = it->first.substr(root.size() + 1);
  //     xml += "<member><name>" + label + "</name>" + it->second.toXml() + "</member>";
  //   }
  // }
  // xml += "</struct></value>";

  // int                 offset = 0;
  // XmlRpc::XmlRpcValue ret(xml, &offset);
  // return ret;

  XmlRpc::XmlRpcValue ret;
  for (Params::iterator it = list.begin(); it != list.end(); it++) {
    if ((root == "") || (it->first.find(root) != std::string::npos)) {
      ret[it->first.substr(label.size())] = it->second;
    }
  }
  return ret;
}

std::ostream& operator<<(std::ostream &stream, const Params& p) {
  XmlRpc::XmlRpcValue value;
    
  for (Params::const_iterator it = p.begin(); it != p.end(); it++) {
    value = (XmlRpc::XmlRpcValue) it->second;
    switch (value.getType()) {
      case XmlRpc::XmlRpcValue::TypeBoolean:
        stream << it->first << ": " << bool(value) << std::endl;
      break;
      case XmlRpc::XmlRpcValue::TypeInt:
        stream << it->first << ": " << int(value) << std::endl;
      break;
      case XmlRpc::XmlRpcValue::TypeDouble:
        stream << it->first << ": " << double(value) << std::endl;
      break;
      case XmlRpc::XmlRpcValue::TypeString:
        stream << it->first << ": " << std::string(value) << std::endl;
      break;
      case XmlRpc::XmlRpcValue::TypeArray:
        stream << it->first << ": {";
        for (int itArray = 0; itArray < value.size(); itArray++) {
          stream << std::string(value[itArray]);
          if (itArray != value.size() - 1)
            stream << ", ";
        }
        stream << "}" << std::endl;
      break;
      default:
        stream << it->first << ": [Non-Defined value type]" << std::endl;
      break;
    }
  }

  return stream;
}

int bytes2int(char * b) {
  int conv = 0;

  for (int k = 0, pow = 1; k < sizeof(int); k++, pow *= 256) {
    conv += (*(b + (sizeof(int)-1) - k))*pow;
  }

  return conv;
}

double bytes2double(char * b) {
  union {
    double d;
    char b[sizeof(double)];
  } conv;

  for (int k = 0; k < sizeof(double); k++) {
    conv.b[(sizeof(double)-1)-k] = *(b + k);
  }

  return conv.d;
}


template <class T>
void XmlRpcArray2vector(XmlRpc::XmlRpcValue& x, std::vector<T>& v) {
  // Check that the XmlRpcValue is an array
  if (x.getType() == XmlRpc::XmlRpcValue::Type::TypeArray) {
    v.clear();
    for (unsigned int k = 0; k < x.size(); k++) {
      v.push_back(T(x[k]));
    }
  }
}

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

Params::const_iterator findKey(const Params& map, const std::string& search_for) {
  Params::const_iterator it = map.lower_bound(search_for);
  if (it != map.end()) {
    const std::string& key = it->first;
    if (key.compare(0, search_for.size(), search_for) == 0) // Really a prefix?
      return it;
  }
  return map.end();
}

} // namespace surfr
