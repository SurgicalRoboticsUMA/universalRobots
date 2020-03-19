function cb_pose(src, msg)
    global ROS

    % ***** IMPORTANT NOTE *****
    % 'tf' cannot be processed here for performance purposes
    
    % Update index of the latest message on cache
    if (ROS.idx.ur3_pose == ROS.cache)
        ROS.idx.ur3_pose = 0;
    end
    ROS.idx.ur3_pose     = ROS.idx.ur3_pose + 1;

    % Save the latest message on cache
    ROS.topics.ur3_pose{ROS.idx.ur3_pose} = msg.Data;
