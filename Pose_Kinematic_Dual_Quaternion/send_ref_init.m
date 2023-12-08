function send_ref_init(markerMsg, markerPublisher, p, r)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Fill in the Marker message fields for a line
markerMsg.Header.Stamp = rostime('now');
markerMsg.Header.FrameId = 'world'; % Replace with your frame ID
markerMsg.Type = 8; % Line strip type (see http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)
markerMsg.Action = 1; % Add or modify
markerMsg.Pose.Orientation.W = 1.0; % Example quaternion (no rotation)
markerMsg.Scale.X = 0.01; % Example width of the line
markerMsg.Scale.Y = 0.01; % Example width of the line
markerMsg.Scale.Z = 0.01; % Example width of the line
markerMsg.Color.R = 0.0; % Example color red
markerMsg.Color.G = 1.0; % Example color green
markerMsg.Color.B = 0.0; % Example color blue
markerMsg.Pose.Position.X = p(2); % Example position in X
markerMsg.Pose.Position.Y = p(3); % Example position in Y
markerMsg.Pose.Position.Z = p(4); % Example position in Z
send(markerPublisher, markerMsg);
end

