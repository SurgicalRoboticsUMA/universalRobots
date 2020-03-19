function S = ur3_ini()
    global ROS

    if (isempty(ROS))
        ROS.cache  = 1000;
        rosshutdown;
        rosinit();
    end
    ROS.idx.ur3_pose = 0;
    ROS.topics.ur3_pose = cell(1, ROS.cache);

    S = rossubscriber('/UR3_1/outputs/pose', @cb_pose);
