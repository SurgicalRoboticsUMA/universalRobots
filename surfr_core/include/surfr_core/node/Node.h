/*
 *
 *  surfr_core/node/Node.h
 * 
 *               Generic Device Node class to load and link data with ROS
 *               ----------------------------------------------------------
 *  Begin Date : November 23, 2016
 *  Revision   : January 31, 2019 (rev 26)
 *  Author     : Enrique Bauzano
 *  Email      : ebauzano@uma.es
 *               ----------------------------------------------------------
 *  To-Do List : - If connection to the real robot fails, try to use
 *                 only the MoveIt! simulator.
 *               - Create a thread to automatically run the update()
 *                 method.
 *               - Actually, the origin frame has a fixed parent (world)
 *                 and child (root_link). Child should be dynamically
 *                 selected from the YAML 'transforms' section (i.e. by
 *                 taking the label with a value of 0).
 *               - Change the IK method to allow any input transform.
 *  Done List  : - Some of the main geometry_msgs are now available to use with
 *                 the class: Point, Quaternion, Pose, Twist and Wrench (R26)
 *               - Publishers and subscribers now have a higher buffer to
 *                 prevent the loss of data transmitted (R25)
 *               - Service callbacks have been updated. Now all specific
 *                 message versions call a base function to make the code
 *                 easier to read (R24).
 *               - A robot pose topic now publishes the original data
 *                 provided by hardware, instead of MoveIt! info (R23)
 *               - Added support for Tool Center Point reference frame in
 *                 robot devices (R22)
 *               - Input planner has been integrated. Now an input signal
 *                 can be planned to follow a trajectory towards the
 *                 target (input signal) (R21)
 *               - Multiple fixes on publishing topics. Now it works
 *                 faster and with better integration of simulator (R20)
 *               - Now the simulation mode is independant from the
 *                 device class (R19)
 *               - Fixed a problem when using simulation mode. Robot
 *                 topics were not correctly updated (R18)
 *               - Added the filter class support to publish filtered
 *                 output signals that have been previously configured
 *                 from a YAML file (R17)
 *               - First version for accessing to MoveIt! IK library
 *                 through a topic subscriber. Actually it uses the
 *                 current robot pose (R16)
 *               - Class has been adapted to allow its use with the ROS
 *                 pluginlib package (R15)
 *               - Joint inputs are now available (R14)
 *               - Assign unique labels to links and joints of the URDF.
 *                 This allows the use of multiple manipulators (R13)
 *               - Remap the robot_description and robot_description_semantic
 *                 parameters into UR3_x/urdf and UR3_x/urdf_semantic (R12)
 *               - Publish of DOF and joint names of the manipulator,
 *                 which are directly provided by the URDF (R11)
 *               - Now the origin frame of reference is passed as param
 *                 to the URDF and synced with the transformBroadcast (R10)
 *               - Correction of the transforms to be compatible with
 *                 the URDF and RVIZ format (R9)
 *               - Publish of MoveIt! jacobian data (R8)
 *               - Publish of MoveIt! pose data (R7)
 *               - Publish of MoveIt! joints data, including positions,
 *                 velocities and names (R6)
 *               - Broadcasting of the Transforms are now available,
 *                 including the origin and links of a manipulator
 *                 device (R5)
 *               - Definition of an optional origin reference frame (R4)
 *               - Merging of Communication and Device node classes (R3)
 *               - Using global YAML parsing function from core.h (R2)
 *               - CommNode class included into surfr::comm namespace (R1)
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

#if !defined(_SURFR_CORE_NODE_H)
#define _SURFR_CORE_NODE_H


// ROS include files
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <pluginlib/class_loader.h>
#include <tf/transform_broadcaster.h>
// ROS messages
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <surfr_msgs/Boolean.h>
#include <surfr_msgs/Unsigned.h>
#include <surfr_msgs/Integer.h>
#include <surfr_msgs/Double.h>
#include <surfr_msgs/String.h>
#include <surfr_msgs/Vector.h>
#include <surfr_msgs/Matrix.h>
#include <surfr_msgs/Joint.h>
#include <tf/transform_datatypes.h>
// ROS services
#include <std_srvs/Empty.h>
#include <surfr_msgs/cmd_empty.h>
#include <surfr_msgs/cmd_boolean.h>
#include <surfr_msgs/cmd_unsigned.h>
#include <surfr_msgs/cmd_integer.h>
#include <surfr_msgs/cmd_double.h>
#include <surfr_msgs/cmd_string.h>
#include <surfr_msgs/cmd_vector.h>
// Surgical Framework include files
#include "surfr_core/Base.h"
#include "surfr_core/filters/Filter.h"
#include "surfr_core/planners/Planner.h"
#include "surfr_core/dev/Manipulator.h"


namespace surfr {

typedef std::map<std::string, ros::Publisher>            Publisher;
typedef std::map<std::string, ros::Subscriber>           Subscriber;
typedef std::map<std::string, ros::ServiceServer>        Service;
typedef std::map<std::string, tf::StampedTransform>      Transforms;
typedef std::map<std::string, surfr::filters::Filter*>   Filters;
typedef std::map<std::string, surfr::planners::Planner*> Planners;

class Node {
  public:
    Node(ros::NodeHandle& nh, boost::shared_ptr<surfr::Base>& dev);
    ~Node();
    void update();
  private:
    const uint32_t _PUBS_BUFFER_SIZE;
    const uint32_t _SUBS_BUFFER_SIZE;

    void _deploy_topics();
    void _publish_topics();
    void _deploy_services();
    template <class Msg>            void _cbSubscriber (const ros::MessageEvent<Msg const>& event);
    template <class Req, class Res> bool _cbService    (ros::ServiceEvent<Req, Res>& event);
                                    bool _cbServiceBase(std::string& name, XmlRpc::XmlRpcValue& data);

    boost::shared_ptr<surfr::Base> _dev;
    ros::Time                      _t0;
    ros::Rate                      _rate;
    bool                           _is_std_msgs;
    ros::NodeHandle*               _nh;
    Publisher                      _pubs;
    Subscriber                     _subs;
    Service                        _srv;
    Params                         _setup;
    tf::TransformBroadcaster*      _tf;
    tf::StampedTransform*          _origin;
    tf::StampedTransform*          _tcp;
    dev::Manipulator*              _robot;
    Transforms                     _frames;
    Filters                        _filters;
    Planners                       _plans;
    Flow                           _in;                                         /*!< Data sent to the device.                   */
  };

template <> void Node::_cbSubscriber(const ros::MessageEvent<sensor_msgs::JointState const>& event);
template <> void Node::_cbSubscriber(const ros::MessageEvent<surfr_msgs::Joint const>& event);
template <> bool Node::_cbService   (ros::ServiceEvent<surfr_msgs::cmd_empty::Request,
                                                       surfr_msgs::cmd_empty::Response>& event);
template <> bool Node::_cbService   (ros::ServiceEvent<surfr_msgs::cmd_vector::Request,
                                                       surfr_msgs::cmd_vector::Response>& event);

} // namespace surfr


#endif  //_SURFR_CORE_NODE_H
