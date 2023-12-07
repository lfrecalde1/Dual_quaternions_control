function [output] = quaternionToAxisAngle(q)
    % Check if the input quaternion is valid
%     if numel(q) ~= 4
%         error('Input quaternion must have four elements.');
%     end
    
    % Normalize the quaternion
    
    q = q/norm(q);
    
    % Extract the scalar and vector parts
    scalarPart = q(1);
    vectorPart = q(2:4);
    
    % Compute the rotation angle (in radians)
    theta = atan2(norm(vectorPart),scalarPart);
    
    % Compute the rotation axis
    axis = vectorPart / norm(vectorPart);
    %axis_aux = vectorPart
    
    output = axis*theta;
    
    vector_part_aux = [0; 0; 1];
    axis_aux = 0;
    if abs(theta) < 2.2204e-15
        
        output = (vectorPart/scalarPart)*(1-((norm(vectorPart)^2)/(3*scalarPart^2)));
        output = vector_part_aux*axis_aux;
        %output = vectorPart;
        
        
    end

end