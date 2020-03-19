function T = ur3_read()

%% INITIALIZATION OF PARAMETERS
    global ROS
    frame      = ROS.topics.ur3_pose{ROS.idx.ur3_pose};
%     DEVID      = 'UR3_1';
%     listLinks  = {'tool39_link', 'tool39_tip', ...
%                   'tool40_link', 'tool40_tip', ...
%                   'tool49_link', 'tool49_tip'};                             % Transform name of all links
%     listHeader = [frame.Header];                                            % General info (name and time)
%     listTime   = [listHeader.Stamp];                                        % Time info (secs & nsecs)
%     listParent = {listHeader.FrameId};                                      % Parent frame names
%     listChild  = {frame.ChildFrameId};                                      % Child frames
%     listTf     = [frame.Transform];                                         % Transforms data (pose)

    % Checks which transforms belong to DEVID device
%     if (sum(~cellfun(@isempty, strfind(listParent, DEVID))) > 0)
%         for kLink = 1:length(listLinks)
%             % Index of the current link on the list of 'tf' topic data
%             idx   = ~cellfun(@isempty, strfind(listChild, listLinks{kLink}));
%             if (sum(idx) > 0)
%                 % Record instant of this frame (in seconds)
%                 T.(listLinks{kLink}).time = double(listTime(idx).Sec) + ...
%                                             1e-9*double(listTime(idx).Nsec);
%                 % Position of the current finger-bone link
%                 P    = [listTf(idx).Translation.X
%                         listTf(idx).Translation.Y
%                         listTf(idx).Translation.Z];
%                 T.(listLinks{kLink}).position = P;
%                 % Orientation of the current finger-bone link
%                 Q    = [listTf(idx).Rotation.W
%                         listTf(idx).Rotation.X
%                         listTf(idx).Rotation.Y
%                         listTf(idx).Rotation.Z];
%                 T.(listLinks{kLink}).orientation = Q;
%                 T.(listLinks{kLink}).transform = [quat2rotm(Q'), P];
%             end
%         end
        ang = norm(frame(4:6));
        T = [axang2rotm([frame(4:6)'/ang, ang]), frame(1:3)];
%     end
