/*
 *
 *  surfr_core/datatypes.h
 * 
 *               Generic variable type for Surgical Framework
 *               ----------------------------------------------------------
 *  Begin Date : May 17, 2017
 *  Revision   : July 24, 2018 (rev 6)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - Use of Eigen::MatrixXd for matricial operations.
 *               - Use of maps integrated with Scalar class.
 *               - New name (Variable?)
 *               - Use of try-catch-throw for exceptions handling.
 *               - Full review of Doxygen documentation.
 *  Done List  : - Fixed behavior of elemental operations (+,-,*,/) (R6)
 *               - ...
 *               - Initial release (R1)
 *               ----------------------------------------------------------
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


#if !defined(_SURFR_CORE_DATATYPES_H)
#define _SURFR_CORE_DATATYPES_H

#include <map>                                    // Standard C++ map class
#include <string>                                 // Standard C++ string class
#include <XmlRpcValue.h>                          // Generic value type of device params
#include <boost/any.hpp>                          // Generic type of a parameter
#include <Eigen/Eigen>                            // Maths with vectors and matrices


namespace surfr {
  class Scalar {
  public:
    typedef enum {
      TYPE_UNDEFINED = 0,
      TYPE_BOOLEAN   = 1,
      TYPE_UNSIGNED  = 2,
      TYPE_INTEGER   = 3,
      TYPE_DOUBLE    = 4,
      TYPE_STRING    = 5
    } Type;

/**
 * Overloaded constructor/destructor versions:
 *   1) No parameters       --> undefined value
 *   2) Templated parameter --> can be a valid C++ numeric type (bool, uint64_t, int64_t or double_t)
 *   3) std::string
 *   4) char* array pointer
 */
    template <class T>
    Scalar(const T&    val) { *this = const_cast<T&>(val); };
    Scalar(const char* str) { *this = std::string(str); };
    Scalar() :
      _boolean(false),
      _usigned(0),
      _integer(0),
      _real(0.0),
      _text(""),
      _type(TYPE_UNDEFINED) { };
    ~Scalar() { };
/**
 * Overloaded assignment operator (=):
 *   1) Templated parameter    --> can be a valid C++ numeric type (bool, unsigned, integer or float)
 *   2) Another Scalar object
 *   3) An XmlRpc::XmlRpcValue
 *   4) std::string
 *   5) char* array pointer
 */
    template <class T>
    Scalar& operator=(const T& val) {
      _text    = std::to_string(val);
                        //  _real    = double_t(val);
      _real    = double_t(strtod(_text.c_str(), NULL));
      _integer = int64_t(_real);
                        //  _usigned = uint64_t(std::abs(_integer));
      _usigned = uint64_t(_integer);
      _boolean = bool(_real);
      if         (std::is_same<T, bool>::value) {
        _type    = TYPE_BOOLEAN;
        _text    = _boolean ? "true" : "false";
      } else if ((std::is_same<T, uint16_t>::value) ||
                 (std::is_same<T, uint32_t>::value) ||
                 (std::is_same<T, uint64_t>::value)) {
        _type    = TYPE_UNSIGNED;
      } else if ((std::is_same<T, int16_t>::value) ||
                 (std::is_same<T, int32_t>::value) ||
                 (std::is_same<T, int64_t>::value)) {
        _type    = TYPE_INTEGER;
      } else if ((std::is_same<T, float_t>::value) ||
                 (std::is_same<T, double_t>::value)) {
        _type    = TYPE_DOUBLE;
      // } else if (std::is_same<T, std::string>::value) {
      //   _type    = TYPE_STRING;
      } else {
        _type    = TYPE_UNDEFINED;
      }
      return *this;
    };
    Scalar& operator=(const Scalar& scalar) {
      _boolean = scalar._boolean;
      _usigned = scalar._usigned;
      _integer = scalar._integer;
      _real    = scalar._real;
      _text    = scalar._text;
      _type    = scalar._type;
      return *this;
    };
    Scalar& operator=(const XmlRpc::XmlRpcValue& xml) {
      switch (xml.getType()) {
      case XmlRpc::XmlRpcValue::Type::TypeBoolean:
        *this = bool(const_cast<XmlRpc::XmlRpcValue&>(xml));
        break;
      case XmlRpc::XmlRpcValue::Type::TypeInt:
        *this = int64_t(int(const_cast<XmlRpc::XmlRpcValue&>(xml)));
        break;
      case XmlRpc::XmlRpcValue::Type::TypeDouble:
        *this = double_t(double(const_cast<XmlRpc::XmlRpcValue&>(xml)));
        break;
      case XmlRpc::XmlRpcValue::Type::TypeString:
        *this = std::string(const_cast<XmlRpc::XmlRpcValue&>(xml));
        break;
      default:
        *this = Scalar();
        break;
      }
      return *this;
    };
    Scalar& operator=(const std::string& str) {
      _text    = str;
      _real    = double_t(strtod(_text.c_str(), NULL));
      _integer = int64_t(_real);
      _usigned = uint64_t(_integer);
      _boolean = bool(_usigned);
      _type    = TYPE_STRING;
      return *this;
    };
    Scalar& operator=(const char* str) {
      *this = std::string(str);
      return *this;
    };
/**
 * Overloaded comparison operators:
 *   1) Scalar {==, !=, >, <, >=, <=} Template value
 *   2) Scalar {==, !=, >, <, >=, <=} Scalar
 *   3) Scalar {==, !=, >, <, >=, <=} char* array
 */
    template <class T> bool operator==(const T& val) {
                         bool ret;
                         if         (std::is_same<T, bool>::value) {
                           ret = (this->_boolean == val);
                         } else if ((std::is_same<T, uint16_t>::value) ||
                                    (std::is_same<T, uint32_t>::value) ||
                                    (std::is_same<T, uint64_t>::value)) {
                           ret = (this->_usigned == val);
                         } else if ((std::is_same<T, int16_t>::value) ||
                                    (std::is_same<T, int32_t>::value) ||
                                    (std::is_same<T, int64_t>::value)) {
                           ret = (this->_integer == val);
                         } else if ((std::is_same<T, float_t>::value) ||
                                    (std::is_same<T, double_t>::value)) {
                           ret = (this->_real == val);
                         } else if (std::is_same<T, std::string>::value) {
                           ret = !(this->_text.compare(val));
                         } else {
                           ret = false;
                         }
                         return ret;
                       };
                       bool operator==(const Scalar& scalar) {
                         return !(this->_text.compare(std::string(scalar)));
                       };
                       bool operator==(const char* str) {
                         return !(this->_text.compare(str));
                       };
    template <class T> bool operator>(const T& val) {
                         bool ret;
                         if         (std::is_same<T, bool>::value) {
                           ret = (this->_boolean > val);
                         } else if ((std::is_same<T, uint16_t>::value) ||
                                    (std::is_same<T, uint32_t>::value) ||
                                    (std::is_same<T, uint64_t>::value)) {
                           ret = (this->_usigned > val);
                         } else if ((std::is_same<T, int16_t>::value) ||
                                    (std::is_same<T, int32_t>::value) ||
                                    (std::is_same<T, int64_t>::value)) {
                           ret = (this->_integer > val);
                         } else if ((std::is_same<T, float_t>::value) ||
                                    (std::is_same<T, double_t>::value)) {
                           ret = (this->_real > val);
                         } else if (std::is_same<T, std::string>::value) {
                           ret = (this->_text.compare(val) > 0);
                         } else {
                           ret = false;
                         }
                         return ret;
                       };
                       bool operator>(const Scalar& scalar) {
                         return (this->_text.compare(std::string(scalar)) > 0);
                       };
                       bool operator>(const char* str) {
                         return (this->_text.compare(str) > 0);
                       };
    template <class T> bool operator!=(const T& val)         { return !(*this == val); };
                       bool operator!=(const Scalar& scalar) { return !(*this == scalar); };
                       bool operator!=(const char* str)      { return !(*this == str); };
    template <class T> bool operator< (const T& val)         { return !(*this > val); };
                       bool operator< (const Scalar& scalar) { return !(*this > scalar); };
                       bool operator< (const char* str)      { return !(*this > str); };
    template <class T> bool operator>=(const T& val)         { return ((*this > val) || (*this == val)); };
                       bool operator>=(const Scalar& scalar) { return ((*this > scalar) || (*this == scalar)); };
                       bool operator>=(const char* str)      { return ((*this > str) || (*this == str)); };
    template <class T> bool operator<=(const T& val)         { return ((*this < val) || (*this == val)); };
                       bool operator<=(const Scalar& scalar) { return ((*this < scalar) || (*this == scalar)); };
                       bool operator<=(const char* str)      { return ((*this < str) || (*this == str)); };
/**
 * Overloaded logical operators:
 *   1) !Scalar
 *   2) Scalar {&&, ||} Template value
 *   3) Scalar {&&, ||} Scalar
 */
    template <class T> bool operator&&(const T& val)         { return (this->_boolean && val); };
                       bool operator&&(const Scalar& scalar) { return (this->_boolean && bool(scalar)); };
    template <class T> bool operator||(const T& val)         { return (this->_boolean && val); };
                       bool operator||(const Scalar& scalar) { return (this->_boolean && bool(scalar)); };
                       bool operator! ()                     { return !(this->_boolean); };

/**
 * Overloaded sum operator (+):
 * NOTE: all (+) operations are implemented in both orders
 *   1) Scalar + Scalar --> The result is always of the type of the left Scalar value
 *   2) Scalar + Template value (bool, uint64_t, int64_t, double_t, std::string)
 *      Template value (bool, uint64_t, int64_t, double_t, std::string) + Scalar
 *      Scalar + std::string --> If Scalar type is std::string, concatenate strings. Otherwise, sum values
 *      std::string + Scalar
 *   3) Scalar + char* array --> Conversion from char* to std::string and cast Scalar + std::string
 *      char* array + Scalar
 */
    friend Scalar operator+(const Scalar& s1, const Scalar& s2) {
      Scalar ret;

      switch (s1.type()) {
      case TYPE_BOOLEAN:
        ret = bool(bool(s1) + bool(s2));
        break;
      case TYPE_UNSIGNED:
        ret = unsigned(unsigned(s1) + unsigned(s2));
        break;
      case TYPE_INTEGER:
        ret = int(int(s1) + int(s2));
        break;
      case TYPE_DOUBLE:
        ret = double(double(s1) + double(s2));
        break;
      case TYPE_STRING:
        ret = std::string(s1) + std::string(s2);
        break;
      }
      return ret;
    };
    template <class T>
    friend Scalar operator+(const Scalar& scalar, const T& val) {
      return (scalar + Scalar(val));
    };
    template <class T>
    friend Scalar operator+(const T& val, const Scalar& scalar) {
      return (Scalar(val) + scalar);
    };
    friend Scalar operator+(const Scalar& scalar, const char* str) {
      return (scalar + Scalar(str));
    };
    friend Scalar operator+(const char* str, const Scalar& scalar) {
      return (Scalar(str) + scalar);
    };
/**
 * Overloaded minus operator (-):
 * NOTE: all (-) operations are implemented in both orders
 *   1) Scalar - Scalar --> The result is always of the type of the left Scalar
 *   2) Scalar - Template value (bool, uint64_t, int64_t, double_t, std::string)
 *      Template value (bool, uint64_t, int64_t, double_t, std::string) - Scalar
 *      Scalar - std::string --> If both are values, substract them and convert to Scalar type.
 *                               Otherwise, delete from scalar string all occurrences of std::strinv
 *      std::string - Scalar
 *   3) Scalar - char* array --> Conversion from char* to std::string and cast Scalar - std::string
 *      char* array - Scalar
 */
    friend Scalar operator-(const Scalar& s1, const Scalar& s2) {
      Scalar ret;

      switch (s1.type()) {
      case TYPE_BOOLEAN:
        ret = bool(bool(s1) - bool(s2));
        break;
      case TYPE_UNSIGNED:
        ret = unsigned(unsigned(s1) - unsigned(s2));
        break;
      case TYPE_INTEGER:
        ret = int(int(s1) - int(s2));
        break;
      case TYPE_DOUBLE:
        ret = double(double(s1) - double(s2));
        break;
      case TYPE_STRING:
        std::string str1 = std::string(s1),
                    str2 = std::string(s2);
        while (str1.find(str2) != std::string::npos) {
          str1 = str1.replace(str1.find(str2), str2.length(), "");
        }
        break;
      }
      return ret;
    };
    template <class T>
    friend Scalar operator-(const Scalar& scalar, const T& val) {
      return (scalar - Scalar(val));
    };
    template <class T>
    friend Scalar operator-(const T& val, const Scalar& scalar) {
      return (Scalar(val) - scalar);
    };
    friend Scalar operator-(const Scalar& scalar, const char* str) {
      return (scalar - std::string(str));
    }
    friend Scalar operator-(const char* str, const Scalar& scalar) {
      return (std::string(str) - scalar);
    }
/**
 * Overloaded product operator (*):
 * NOTE: all (*) operations are implemented in both orders
 *   1) Scalar1 * Scalar2
 *   2) Scalar * Template value (bool, uint64_t, int64_t or double_t)
 *      Template value * Scalar
 */
    friend Scalar operator*(const Scalar& s1, const Scalar& s2) {
      Scalar ret;

      switch (s1.type()) {
      case TYPE_BOOLEAN:
        ret = bool(bool(s1) * bool(s2));
        break;
      case TYPE_UNSIGNED:
        ret = unsigned(unsigned(s1) * unsigned(s2));
        break;
      case TYPE_INTEGER:
        ret = int(int(s1) * int(s2));
        break;
      case TYPE_DOUBLE:
        ret = double(double(s1) * double(s2));
        break;
      case TYPE_STRING:
// throw an exception here?
        break;
      }
      return ret;
    };
    template <class T>
    friend Scalar operator*(const Scalar& scalar, const T& val) {
      return (scalar * Scalar(val));
    };
    template <class T>
    friend Scalar operator*(const T& val, const Scalar& scalar) {
      return (Scalar(val) * scalar);
    };
/**
 * Overloaded product operator (/):
 * NOTE: all (/) operations are implemented in both orders
 *   1) Scalar1 / Scalar2
 *   2) Scalar / Template value (bool, uint64_t, int64_t or double_t)
 *      Template value / Scalar
 */
    friend Scalar operator/(const Scalar& s1, const Scalar& s2) {
      Scalar ret;

      switch (s1.type()) {
      case TYPE_BOOLEAN:
        ret = bool(s1) && bool(s2);
        break;
      case TYPE_UNSIGNED:
        ret = unsigned(unsigned(s1) / unsigned(s2));
        break;
      case TYPE_INTEGER:
        ret = int(int(s1) / int(s2));
        break;
      case TYPE_DOUBLE:
        ret = double(double(s1) / double(s2));
        break;
      case TYPE_STRING:
// throw an exception here?
        break;
      }
      return ret;
    };
    template <class T>
    friend Scalar operator/(const Scalar& scalar, const T& val) {
      return (scalar / Scalar(val));
    };
    template <class T>
    friend Scalar operator/(const T& val, const Scalar& scalar) {
      return (Scalar(val) / scalar);
    };
/**
 * Overloaded product operator (%):
 * NOTE: all (%) operations are implemented in both orders
 *   1) Scalar1 % Scalar2
 *   2) Scalar % Template value (bool, uint64_t, int64_t or double_t)
 *      Template value % Scalar
 */
    friend Scalar operator%(const Scalar& s1, const Scalar& s2) {
      Scalar ret;

      switch (s1.type()) {
      case TYPE_BOOLEAN:
        ret = bool(s1) && bool(s2);
        break;
      case TYPE_UNSIGNED:
        ret = unsigned(unsigned(s1) % unsigned(s2));
        break;
      case TYPE_INTEGER:
        ret = int(int(s1) % int(s2));
        break;
      case TYPE_DOUBLE:
        ret = double(int(s1) % int(s2));
        break;
      case TYPE_STRING:
// throw an exception here?
        break;
      }
      return ret;
    };
    template <class T>
    friend Scalar operator%(const Scalar& scalar, const T& val) {
      return (scalar % Scalar(val));
    };
    template <class T>
    friend Scalar operator%(const T& val, const Scalar& scalar) {
      return (Scalar(val) % scalar);
    };
/**
 * Overloaded compound assignment operators (+=, -=, *=, /=):
 *   1) Scalar += Template value (bool, uint64_t, int64_t or double_t)
 *   2) Scalar += char* array
 *   3) Scalar -= Template value (bool, uint64_t, int64_t or double_t)
 *   4) Scalar *= Template value (bool, uint64_t, int64_t or double_t)
 *   5) Scalar /= Template value (bool, uint64_t, int64_t or double_t)
 *   6) Scalar %= Template value (bool, uint64_t, int64_t or double_t)
 */
    template <class T> Scalar& operator+=(const T& val)    { *this = *this + val; return *this; };
                       Scalar& operator+=(const char* str) { *this = *this + str; return *this; };
    template <class T> Scalar& operator-=(const T& val)    { *this = *this - val; return *this; };
                       Scalar& operator-=(const char* str) { *this = *this - str; return *this; };
    template <class T> Scalar& operator*=(const T& val)    { *this = *this * val; return *this; };
    template <class T> Scalar& operator/=(const T& val)    { *this = *this / val; return *this; };
    template <class T> Scalar& operator%=(const T& val)    { *this = *this % val; return *this; };
/**
 * Overloaded increment-decrement operators (++, --):
 *   1) ++Scalar
 *   2) Scalar++
 *   3) --Scalar
 *   4) Scalar--
 */
                       Scalar& operator++()    { *this += 1; return *this; };                       // Prefix  (++Scalar)
                       Scalar& operator++(int) { Scalar& ret(*this); ++(*this); return ret; };      // Postfix (Scalar++)
                       Scalar& operator--()    { *this -= 1; return *this; };                       // Prefix  (--Scalar)
                       Scalar& operator--(int) { Scalar& ret(*this); --(*this); return ret; };      // Postfix (Scalar--)
/**
 * Overloaded stream operator (<<):
 */
                       friend std::ostream& operator<<(std::ostream &stream, const Scalar& scalar);
/**
 * Overloaded casting operators (both const and non-const versions):
 *   1) bool
 *   2) unsigned
 *   3) int
 *   4) double
 *   5) std::string
 *   6) XmlRpc::XmlRpcValue
 */
                       operator bool&        () { return _boolean; };
                       operator bool&        () const { return const_cast<bool&>(_boolean); };
                       operator unsigned     () { return _usigned; };
                       operator unsigned     () const { return const_cast<uint64_t&>(_usigned); };
                       operator int          () { return _integer; };
                       operator int          () const { return const_cast<int64_t&>(_integer); };
                       operator double&      () { return _real;    };
                       operator double&      () const { return const_cast<double_t&>(_real);    };
                       operator std::string& () { return _text;    };
                       operator std::string& () const { return const_cast<std::string&>(_text); };
                       operator XmlRpc::XmlRpcValue () {
                         switch (_type) {
                         case TYPE_BOOLEAN:
                           return XmlRpc::XmlRpcValue(bool(_boolean));
                         case TYPE_UNSIGNED:
                         case TYPE_INTEGER:
                           return XmlRpc::XmlRpcValue(int(_integer));
                         case TYPE_DOUBLE:
                           return XmlRpc::XmlRpcValue(double(_real));
                         case TYPE_STRING:
                           return XmlRpc::XmlRpcValue(std::string(_text));
                         default:
                           return XmlRpc::XmlRpcValue();
                         }
                       };
/**
 * Class methods
 */
                       Type        type()             { return _type; };
                       Type        type() const       { return _type; };
                       void        type(Type type)    { _type = type; };
                       std::string type_desc()        { return _type_desc[_type]; };
                       bool        ok()               { return (_type != TYPE_UNDEFINED); }
                       bool        is_text() const    { return (_type == TYPE_STRING); };
                       bool        is_numeric() const { return (_real != 0.0); };
                      //  double_t    value() const      { return _real; };
                      //  std::string text()  const      { return _text; };
/**
 * Attributes
 */
  private:
    bool        _boolean;
    uint64_t    _usigned;
    int64_t     _integer;
    double_t    _real;
    std::string _text;
    Type        _type;

    const std::vector<std::string> _type_desc = {"undefined",
                                                 "boolean",
                                                 "unsigned",
                                                 "integer",
                                                 "double",
                                                 "string"};
  };
// /**
//  * class Scalar: Templates specializations
//  */
//   template <> Scalar::Scalar(const std::string& str)         { *this = str; }
//   template <> Scalar::Scalar(const XmlRpc::XmlRpcValue& xml) { *this = const_cast<XmlRpc::XmlRpcValue&>(xml); }
/**
 * class Scalar: Friend overloaded operators
 */
  std::ostream& operator<<(std::ostream &stream, const Scalar& scalar) {
    stream << std::string(scalar);
    return stream;
  }
    

  // class Vector {
  // public:
  //   Vector()                     { _type = Scalar::Type::TYPE_UNDEFINED; };
  //   Vector(const Vector& vector) { *this = vector; };
  //   template <class T> Vector(const std::vector<T>& vector) { *this = vector; };
  //   template <typename T, unsigned S> Vector(const T (&array)[S]) { *this = array; };
  //   template <class T> Vector(const uint64_t size = 1, const T& value = T(0)) :
  //     _value(size, Scalar(value)) {
  //     _type = _value.back().type();
  //   };
  //   ~Vector() {};
  //   Vector& operator=(const Vector& vector) {
  //     _value.clear();
  //     _value = std::vector<Scalar>(vector);
  //     _type = vector.type();
  //     return *this;
  //   };
  //   template <class T> Vector& operator=(const T& value) {
  //     _value.clear();
  //     _value.push_back(Scalar(value));
  //     _type = _value.back().type();
  //     return *this;
  //   };
  //   template <class T> Vector& operator=(const std::vector<T>& vector) {
  //     _value.clear();
  //     for (uint64_t k = 0; k < vector.size(); k++) {
  //       _value.push_back(Scalar(vector.at(k)));
  //     }
  //     _type = _value.back().type();
  //     return *this;
  //   };
  //   template <typename T, unsigned S> Vector& operator=(const T (&array)[S]) {
  //     _value.clear();
  //     for (uint64_t k = 0; k < S; k++) {
  //       _value.push_back(Scalar(array[k]));
  //     }
  //     _type = _value.back().type();
  //     return *this;
  //   }

  //   Scalar const& operator[](const uint64_t& index) const { return const_cast<Scalar &>(_value.at(index)); };
  //   Scalar&       operator[](const uint64_t& index)       { return _value.at(index); };
  //                 operator std::vector<Scalar>& () const { return const_cast<std::vector<Scalar>&>(_value);};
  //   Scalar::Type type() const { return _type; };
  //   int          size() const { return _value.size(); };
  // private:
  //   Scalar::Type        _type;
  //   std::vector<Scalar> _value;
  // };

} // namespace surfr

#endif  //_SURFR_CORE_DATATYPES_H
