function [line_y, line_x, m] = trace_line(m, initial_y, initial_x, color_to_follow, path_color, slow, jump)
    % trace_line: follow the line, paint each point to a different color,
    % and store the points one by one until no more points of the color
    % COLOR_TO_FOLLOW exist in the current neighborhood.
    % assumes the curve to follow is composed of one-pixel width, continuous,
    % (no gaps) 8-connected curve.
    %
    % input:
    %  m - matrix, the map with lines to trace.
    %  initial_y, initial_x - scalars, the starting point for the trace.
    %  color_to_follow - scalar, the color of the line to trace.
    %  path_color - scalar, the color to paint the traced line.
    %  slow - Boolean, if true, use presentation mode.
    %  jump - scalar, how many steps to jump in slow mode.
    % output:
    %  line_y, line_x - 1d arrays, the ordered positions of the path to follow.
    %  m - matrix, the color map with the traced line painted on it.
 
    % Initialize the output arrays with the initial point.
    line_y = initial_y;
    line_x = initial_x;
 
    % Set the initial point to the path color.
    m(initial_y, initial_x) = path_color;
 
    % Define the 8-connected neighborhood.
    neighbors = [-1,  0; % up
                 1,  0; % down
                 0, -1; % left
                 0,  1; % right
                -1, -1; % up-left
                -1,  1; % up-right
                 1, -1; % down-left
                 1,  1];% down-right
 
    % Initialize the current position.
    current_y = initial_y;
    current_x = initial_x;
 
    slow_counter = 0;
 
    % Continue tracing until no more neighbors of the color_to_follow are found.
    while true
        found = false;
        for i = 1:size(neighbors, 1)
            ny = current_y + neighbors(i, 1);
            nx = current_x + neighbors(i, 2);
 
            % Check if the neighbor is within bounds and has the color_to_follow.
            if ny > 0 && ny <= size(m, 1) && nx > 0 && nx <= size(m, 2) && m(ny, nx) == color_to_follow
                % Update the current position.
                current_y = ny;
                current_x = nx;
 
                % Paint the current position with the path color.
                m(current_y, current_x) = path_color;
 
                % Append the current position to the output arrays.
                line_y(end + 1) = current_y; %#ok<AGROW>
                line_x(end + 1) = current_x; %#ok<AGROW>
 
                found = true;
                break;
            end
        end
 
        % If no neighbor with the color_to_follow is found, break the loop.
        if ~found
            break;
        end
 
        if slow
            slow_counter = slow_counter + 1;
            if slow_counter >= jump
                slow_counter = 0;
                imagesc(m);
                axis equal;
                axis tight;
                axis off;
                drawnow;
            end
        end
    end
end
