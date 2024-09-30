function [line_y, line_x, m] = trace_border(m, initial_y, initial_x,...
      initial_dy, initial_dx, color_1, color_2, border_path_color, slow, jump)
   % trace_border finds the border line between two colors (color_1 and color_2),
   % and return this line as a set of positions. all border points are
   % points that are of one color and have neighbors of the other color.
   % the function starts from the initial point that must be the first point of
   % the border. the color at this location must be color_1 or color_2,
   % and it must have neighbors of both colors. the function then starts
   % to trace the border in the initial direction (initial_dx, initial_dy),
   % the direction to the next point along the border.
   % the function continues to trace until no border points remain.
   %
   % input:
   %  m
   %    matrix, the map with the two colors.
   %
   %  initial_y,initial_x
   %    scalars, the starting point for the trace.
   %
   %  initial_dy,initial_dx
   %    scalars, the initial direction of the border
   %    i.e. the increment from the first point of the border line to the
   %    next point.
   %
   %  color_1, color_2,
   %    scalars, the two colors. the border points are pixels in the map m
   %    that are of one color and have neighbors of the other color.
   %
   %  border_path_color,
   %    scalar, the color to paint the traced border line.
   %
   %  slow,
   %    Boolean, if == 1, use presentation mode.
   %
   %  jump
   %    scalar, how many steps to jump in slow mode
   %
   % output:
   %  line_y, line_x
   %    1d arrays, the ordered positions of the border between the two colors.
   %
   %  m
   %    matrix, the color map with the traced border painted on it.
   
    % Initialize the output arrays with the initial point.
    line_y = initial_y;
    line_x = initial_x;
    
    % Set the initial point to the border path color.
    m(initial_y, initial_x) = border_path_color;
    
    % Define the 8-connected neighborhood.
    neighbors = [-1,  0; % up
                 1,  0; % down
                 0, -1; % left
                 0,  1; % right
                -1, -1; % up-left
                -1,  1; % up-right
                 1, -1; % down-left
                 1,  1];% down-right
             
    % Initialize the current position and direction.
    current_y = initial_y;
    current_x = initial_x;
    dy = initial_dy;
    dx = initial_dx;
    
    slow_counter = 0;
    
    while true
        found = false;
        for i = 1:size(neighbors, 1)
            ny = current_y + dy;
            nx = current_x + dx;
            
            % Check if the neighbor is within bounds and is a border point.
            if ny > 0 && ny <= size(m, 1) && nx > 0 && nx <= size(m, 2)
                if (m(ny, nx) == color_1 && has_neighbor(m, ny, nx, color_2, neighbors)) || ...
                   (m(ny, nx) == color_2 && has_neighbor(m, ny, nx, color_1, neighbors))
                    % Update the current position.
                    current_y = ny;
                    current_x = nx;
                    
                    % Paint the current position with the border path color.
                    m(current_y, current_x) = border_path_color;
                    
                    % Append the current position to the output arrays.
                    line_y(end + 1) = current_y; %#ok<AGROW>
                    line_x(end + 1) = current_x; %#ok<AGROW>
                    
                    found = true;
                    break;
                end
            end
            
            % Update direction.
            dy = neighbors(i, 1);
            dx = neighbors(i, 2);
        end
        
        % If no border point is found, break the loop.
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

function result = has_neighbor(m, y, x, color, neighbors)
    % Check if the point (y, x) has a neighbor with the given color.
    result = false;
    for i = 1:size(neighbors, 1)
        ny = y + neighbors(i, 1);
        nx = x + neighbors(i, 2);
        if ny > 0 && ny <= size(m, 1) && nx > 0 && nx <= size(m, 2) && m(ny, nx) == color
            result = true;
            return;
        end
    end
end
