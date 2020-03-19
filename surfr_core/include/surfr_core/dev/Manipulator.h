/*
 *
 *  surfr_core/dev/Manipulator.h
 * 
 *               Robot Manipulator Model class for Surgical Framework
 *               ---------------------------------------------------------------
 *  Begin Date : December 23, 2016
 *  Revision   : November 8, 2018 (rev 6)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ---------------------------------------------------------------
 *  To-Do List : - The velocity_joints() and acceleration_joints() have
 *                 been temporarily removed for backwards compatibility
 *                 purposes (R6)
 *               - Use multiple groups to allow handling various
 *                 manipulators within the same class (i.e. humanoids or
 *                 fingers).
 *               - Create a thread to automatically run the update()
 *                 method.
 *               - Actually, the origin frame has a fixed parent (world)
 *                 and child (root_link). Child should be dynamically
 *                 selected from the YAML 'transforms' section (i.e. by
 *                 taking the label with a value of 0).
 *               - MoveIt! IK library must be accesed to compute the
 *                 joints position of a given cartesian pose.
 *  Done List  : - Now the origin reference frame is get from the robot
 *                 model data (R5)
 *               - Transforms of each arm link related to its parent (R4)
 *               - Obtention of end effector pose (R3)
 *               - Obtention of joints data (R2)
 *               - Connection with one specific MoveIt! group and creation
 *                 of a RobotState element (R1)
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
 *               ---------------------------------------------------------------
 */


#if !defined(_SURFR_CORE_MANIPULATOR_H)
#define _SURFR_CORE_MANIPULATOR_H

#include "surfr_core/core.h"
// MoveIt! Library
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


namespace surfr { namespace dev {
  const uint8_t GLOBAL_FRAME = 0;
  const uint8_t LOCAL_FRAME  = 1;

  /*! \brief Parent class for all the drivers programmed for the devices available on the Medical
   * Robotics Framework.
   *
   *  This class includes all the common methods that must be present in all the drivers implemented
   *  for the considered on the Medical Robotics Platform.
   */
  class Manipulator {
  public:
    Manipulator(std::string desc);
    ~Manipulator();

    void                     control(bool control);
    bool                     control();
    uint16_t                 dof();
    const std::string        frame();
    std::vector<std::string> links_name();
    void                     update(std::vector<double_t>& joints);
    std::vector<std::string> joints_name();
    std::vector<double_t>    joints_position();
    //~ std::vector<double_t>    joints_velocity();
    //~ std::vector<double_t>    joints_acceleration();
    std::vector<double_t>    joints_ik(Eigen::Affine3d& transform);
    std::vector<double_t>    joints_ik(const std::vector<double_t>& pose);
    std::vector<double_t>    pose(const uint8_t& mode = GLOBAL_FRAME);
    Eigen::Affine3d          transform(const std::string &link = "");
    Eigen::MatrixXd          jacobian();
    //~ friend std::ostream& operator<<(std::ostream& stream, const DeviceBase& dev);

/* INCLUIR EN EL FUTURO:
 * - Variable de modo depuracion para general texto informativo
 * - Gestion de errores mediante try-catch-throw
 * - Comentarios enlazables por doxygen
 */ 

  protected:
    bool                                  _is_controller;                       // Sets MoveIt! as main robot controller.
    uint16_t                              _nojoints;                            // Number of joints.
    uint16_t                              _nolinks;                             // Number of links.
    robot_model_loader::RobotModelLoader* _loader;                              // Loads the Robot Model.
    robot_model::RobotModelPtr            _model;                               // Contents the Robot Model.
    robot_state::RobotState*              _state;                               // Current state of the Robot Model.
    robot_state::JointModelGroup*         _group;                               // Group of joints from the Robot Model.
    std::vector<std::string>              _joints;                              // Joint values for the current state.
    std::vector<std::string>              _links;                               // Link values for the current state.
    Eigen::Affine3d                       _inv_root;                            // Inverse transform for the root link.

    //~ std::string _message(const std::string& method,
                         //~ const std::string& msg,
                         //~ IssueLevel level = ISSUE_INFO);
  };
} }

#endif  //_SURFR_CORE_MANIPULATOR_H
