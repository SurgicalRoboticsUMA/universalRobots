
#include "surfr_core/filters/Iir2.h"


namespace surfr { namespace filters {

// Iir2<T>::Iir2(double b0, double b1, double a1, double a2):
Iir2::Iir2(XmlRpc::XmlRpcValue params):
  FilterBase(params),
  _x1(Eigen::VectorXd(1)),
  _y1(Eigen::VectorXd(1)),
  _b0(params["b0"]),
  _b1(params["b1"]),
  _a1(params["a1"]),
  _a2(params["a2"]),
  _order(1) {
    // std::cout << "New Iir2 params: [" << _a1 << ", " << _a2 << ", " << _b0 << ", " << _b1 << "]" << std::endl; 
  SURFR_DEBUG("Parameters of the Iir2 filter:");
  SURFR_DEBUG("a1: " + std::to_string(_a1));
  SURFR_DEBUG("a2: " + std::to_string(_a2));
  SURFR_DEBUG("b0: " + std::to_string(_b0));
  SURFR_DEBUG("b1: " + std::to_string(_b1));
  SURFR_DEBUG("Done");
}
  
Eigen::VectorXd Iir2::process(const Eigen::VectorXd& x){
  if (x.size() != _x1.size()) {
    _x1 = x;
    _y1 = x;
    this->reset();
  }

  Eigen::VectorXd y = (_b0*x + _b1*_x1 - _a2*_y1)/_a1;
    _x1 = x;
    _y1 = y;

  // The first "order-1" samples are not filtered
  if (_order > 0) {
    _order--;
    y = x;
  }

  return y;
}
   
void Iir2::reset() {
  _x1 = Eigen::VectorXd::Zero(_x1.size());
  _y1 = Eigen::VectorXd::Zero(_y1.size());

  // For the transient time of first "order-1" samples
  _order = 1;
}

} }