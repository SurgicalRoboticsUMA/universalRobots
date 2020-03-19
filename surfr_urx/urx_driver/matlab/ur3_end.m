function ur3_end()
% ros_close  Closes a customized ROS initialization
%
%    ros_close()
%    Finishes the communication with ROS on Matlab previously initialized
%    by ros_init() function (use 'help ros_init' on the terminal for more
%    information).
%
% See also
%    ros_config (from Medical Robotics Toolbox)
%    rosinit, rosshutdown (from Robotics System Toolbox)

%     global ROS

%% ROS CLOSING
    rosshutdown;
    pause(0.1);
    clear global ROS
%     if (isfield(ROS, 'topics'))
%         ROS = rmfield(ROS, 'idx');
%         ROS = rmfield(ROS, 'topics');
%     end
