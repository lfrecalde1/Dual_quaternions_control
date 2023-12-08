function send_ref(markerMsg, markerPublisher, p, r)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Fill in the Marker message fields for a line
markerMsg.Header.Stamp = rostime('now');
markerMsg.Pose.Orientation.W = 1.0; % Example quaternion (no rotation)
markerMsg.Pose.Position.X = p(2); % Example position in X
markerMsg.Pose.Position.Y = p(3); % Example position in Y
markerMsg.Pose.Position.Z = p(4); % Example position in Z
send(markerPublisher, markerMsg);
end

