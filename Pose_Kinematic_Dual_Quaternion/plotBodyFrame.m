function plotBodyFrame(orientation, translation)
    % Input:
    %   - orientation: Quaternion representing the orientation of the frame
    %   - translation: 3-element vector representing the translation of the frame

    % Create a rotation matrix from the quaternion
    rotationMatrix = quat2rotm(orientation);

    % Define the frame vectors in the local frame
    frameVectors = [1 0 0; 0 1 0; 0 0 1];

    % Rotate and translate the frame vectors to the global frame
    globalFrameVectors = rotationMatrix * frameVectors' + translation';

    % Plot the frame
    hold on;
    
    % X-axis in red
    quiver3(translation(1), translation(2), translation(3), globalFrameVectors(1,1), globalFrameVectors(2,1), globalFrameVectors(3,1), 'r', 'LineWidth', 2);
    
    % Y-axis in green
    quiver3(translation(1), translation(2), translation(3), globalFrameVectors(1,2), globalFrameVectors(2,2), globalFrameVectors(3,2), 'g', 'LineWidth', 2);
    
    % Z-axis in blue
    quiver3(translation(1), translation(2), translation(3), globalFrameVectors(1,3), globalFrameVectors(2,3), globalFrameVectors(3,3), 'b', 'LineWidth', 2);

    % Set axis labels
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Set axis equal for better visualization
    axis equal;
    
    % Set plot properties
    grid on;
    view(3);
    
    hold off;
end