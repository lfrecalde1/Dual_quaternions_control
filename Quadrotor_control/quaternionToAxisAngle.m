function [theta, axis] = quaternionToAxisAngle(q)
    % Check if the input quaternion is valid
    if numel(q) ~= 4
        error('Input quaternion must have four elements.');
    end
    
    % Normalize the quaternion
    q = q/norm(q);
    
    % Extract the scalar and vector parts
    scalarPart = q(1);
    vectorPart = q(2:4);
    
    % Compute the rotation angle (in radians)
    theta = 2 * acos(scalarPart);
    
    % Compute the rotation axis
    axis = vectorPart / norm(vectorPart);
end