function send_odom(odomMsg, odomPublisher, p, r)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Fill in the Odometry message fields
odomMsg.Header.Stamp = rostime('now');
odomMsg.Header.FrameId = 'world';
odomMsg.Pose.Pose.Position.X = p(2); % Example translation in X
odomMsg.Pose.Pose.Position.Y = p(3); % Example translation in Y
odomMsg.Pose.Pose.Position.Z = p(4); % Example translation in Z
odomMsg.Pose.Pose.Orientation.W = r(1); % Example quaternion (no rotation)
odomMsg.Pose.Pose.Orientation.X = r(2); % Example quaternion (no rotation)
odomMsg.Pose.Pose.Orientation.Y = r(3); % Example quaternion (no rotation)
odomMsg.Pose.Pose.Orientation.Z = r(4); % Example quaternion (no rotation)
% Publish the odometry message
send(odomPublisher, odomMsg);

end