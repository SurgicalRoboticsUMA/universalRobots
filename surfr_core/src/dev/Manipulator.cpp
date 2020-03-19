/*
 *
 *  surfr_core/dev/Manipulator.cpp
 * 
 *               Robot Manipulator Model class for Surgical Framework
 *               ---------------------------------------------------------------
 *  Begin Date : December 23, 2016
 *  Revision   : November 8, 2018 (rev 6)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ---------------------------------------------------------------
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
 *               ---------------------------------------------------------------
 */


#include "surfr_core/dev/Manipulator.h"


namespace surfr { namespace dev {

Manipulator::Manipulator(std::string desc):
  _is_controller(false),
  _nojoints(0),
  _nolinks(0),
  _loader(NULL),
  _state(NULL) {

  // Instantiation of a RobotModelLoader object, which will look up the robot description
  // on the ROS parameter server and construct a ::moveit_core::RobotModel for us to use.
  // _loader = new robot_model_loader::RobotModelLoader("robot_description");
  _loader = new robot_model_loader::RobotModelLoader(desc.c_str());
  _model  = _loader->getModel();
  // _model->printModelInfo(std::cout);
      
  const std::vector<std::string> arms = _model->getJointModelGroupNames();
  if (!arms.size()) {
    SURFR_ERROR("MoveIt! robot model must have at least one group defined.");
    return;
  }
// NOTE: Class is not ready to allow multiple groups yet!
  _group  = _model->getJointModelGroup(arms[0]);
  // _group->printGroupInfo();

  // Use of ::moveit_core::RobotModel to construct a ::moveit_core::RobotState that
  // maintains the configuration of the robot. We will set all joints in the state to
  // their default values. We can then get a ::moveit_core::JointModelGroup, which
  // represents the robot model for a particular group.
  _state = new robot_state::RobotState(_model);
  _state->setToDefaultValues();

  _joints   = _group->getActiveJointModelNames();
  _links    = _group->getLinkModelNames();
  _nojoints = _joints.size();
  _nolinks  = _links.size();
/* QUITAR TRAS TESTEO */
// std::vector<double_t> Q = {0.5555, -1.9080, 2.3038, -3.8019, -1.4788, 6.2439};
// this->update(Q);
/* QUITAR TRAS TESTEO */
  _inv_root = this->transform("UR3_1/root_link").inverse();
// std::cout << "Troot: " << std::endl << _inv_root.matrix() << std::endl;
// std::cout << "World Pose: " << std::endl << this->transform().matrix() << std::endl;
// std::cout << "Root Pose: " << std::endl << (_inv_root*this->transform()).matrix() << std::endl;
}

Manipulator::~Manipulator() {
  delete _state;
  delete _loader;
}

void Manipulator::control(bool control) {
  _is_controller = control;
}

bool Manipulator::control() {
  return _is_controller;
}

uint16_t Manipulator::dof() {
  return _nojoints;
}

const std::string Manipulator::frame() {
  return _model->getModelFrame().substr(1).c_str();
}

void Manipulator::update(std::vector<double_t>& joints) {
  _state->setJointGroupPositions(_group, joints);
  // NOTE: setJointGroupPositions() does not enforce joint limits by itself, but a call to
  //       enforceBounds() will do it.
  _state->enforceBounds();
}

std::vector<std::string> Manipulator::joints_name() {
  return _joints;
}

std::vector<double_t> Manipulator::joints_position() {
  std::vector<double_t> joints;
  _state->copyJointGroupPositions(_group, joints);
  return joints;
}

//~ std::vector<double_t> Manipulator::joints_velocity() {
  //~ std::vector<double_t> joints;
  //~ _state->copyJointGroupVelocities(_group, joints);
  //~ return joints;
//~ }
//~ 
//~ std::vector<double_t> Manipulator::joints_acceleration() {
  //~ std::vector<double_t> joints;
  //~ _state->copyJointGroupAccelerations(_group, joints);
  //~ return joints;
//~ }

std::vector<double_t> Manipulator::joints_ik(Eigen::Affine3d& transform) {
  std::vector<double_t> ret(_nojoints);

  bool isIK = _state->setFromIK(_group, this->transform(*(_links.begin()))*transform, 10, 0.1);
  if (isIK) {
    _state->copyJointGroupPositions(_group, ret);
  } else {
    SURFR_DEBUG("Did not find IK solution.");
  }
  return ret;
}

std::vector<double_t> Manipulator::joints_ik(const std::vector<double_t>& pose) {
  Eigen::Vector3d Pik(pose[0], pose[1], pose[2]);
  Eigen::Vector3d Aik(pose[3], pose[4], pose[5]);
  Eigen::Affine3d Tik = Eigen::Translation3d(Pik)*Eigen::Affine3d(Eigen::AngleAxisd(Aik.norm(), Aik/Aik.norm()))*
                        Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, -0.035));
  // std::cout << "T0: " << std::endl << (this->transform(*(_links.begin()))).matrix() << std::endl;
  // std::cout << "Tik: " << std::endl << Tik.matrix() << std::endl;
  // std::cout << "T: " << std::endl << this->transform().matrix() << std::endl;
  // std::cout << "T0*Tik: " << std::endl << (this->transform(*(_links.begin()))*Tik).matrix() << std::endl;
  return this->joints_ik(Tik);
}

std::vector<std::string> Manipulator::links_name() {
  return _links;
}

std::vector<double> Manipulator::pose(const uint8_t& mode) {
  Eigen::Affine3d T = ((mode == LOCAL_FRAME) ? (_inv_root*this->transform()) : this->transform());
    Eigen::Vector3d   P(T.translation());
    Eigen::AngleAxisd R(T.rotation());
    Eigen::Vector3d   A(R.axis()*R.angle());
  std::vector<double> ret = {P(0), P(1), P(2), A(0), A(1), A(2)};
  return ret;
}

Eigen::Affine3d Manipulator::transform(const std::string &link) {
  ros::V_string::iterator l = std::find(_links.begin(), _links.end(), (link == "") ? _links.back() : link);

  // Base link (first) or Tool link (last)
  if        ((l == _links.begin()) || ((*l) == _links.back())) {
    return _state->getGlobalLinkTransform(*l);
  // Requested link not found
  } else if (l == _links.end()) {
    return Eigen::Affine3d(Eigen::Matrix4d::Identity());
  // Any other valid link
  } else {
    // return _state->getGlobalLinkTransform(*(l-1)).inverse()*_state->getGlobalLinkTransform(*l);
    return _state->getGlobalLinkTransform(*l);
  }
}

Eigen::MatrixXd Manipulator::jacobian() {
  return _state->getJacobian(_group);
}

} }
