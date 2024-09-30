function [path_y, path_x, m] = gradient_descent(m, start_y, start_x, ...
      target_color, path_color, slow, jump, num_neighbors)
   % Initialize variables
   start_color = m(start_y, start_x);   
   steps = start_color - target_color;             
   path_x = zeros(1, steps);
   path_y = zeros(1, steps);   
   r = start_y; c = start_x; required_color = m(r, c) - 1;
   slow_counter = 0;
   
   % Main loop to find the path
   for step = 1:steps         
       [r, c, min_value] = next_step(m, r, c, required_color, num_neighbors);
       if min_value == target_color
           break;
       end
       if (r > 0 && c > 0) && (m(r, c) > 0)           
           m(r, c) = path_color; 
           path_x(step) = c;
           path_y(step) = r;
           required_color = min_value - 1;     
           slow_counter = slow_print(m, slow, jump, slow_counter);        
       end
   end    
   drawnow;
end

function [r, c, min_value] = next_step(m, r, c, required_color, num_neighbors)  
    if num_neighbors == 8       
       [r, c, min_value] = next_step_8(m, r, c, required_color);
    else
       [r, c, min_value] = next_step_4(m, r, c, required_color);
    end
end

function slow_counter = slow_print(m, slow, jump, slow_counter)
    if slow
        slow_counter = slow_counter + 1;
        if slow_counter >= jump
           slow_counter = 0;
           imagesc(m);
           colormap jet;
           axis equal;
           axis off;
           drawnow;
        end
    end
end

function [r, c, min_value] = next_step_8(m, row, col, required)       
    r = -1; c = -1;
    min_value = inf;
    
    % Define 8 neighboring positions
    neighbors = [-1, -1; -1, 0; -1, 1; 0, -1; 0, 1; 1, -1; 1, 0; 1, 1];
    
    for i = 1:size(neighbors, 1)
        newRow = row + neighbors(i, 1);
        newCol = col + neighbors(i, 2);
        if is_valid_cell(m, newRow, newCol, required)
            if m(newRow, newCol) < min_value
                min_value = m(newRow, newCol);
                r = newRow;
                c = newCol;
            end
        end
    end
    
    % Fall back to 4 neighbors if no valid cell found
    if r == -1 || c == -1
        [r, c, min_value] = next_step_4(m, row, col, min_value);
    end
end

function [r, c, min_value] = next_step_4(m, row, col, required) 
    r = -1; c = -1;
    min_value = inf;
    
    % Define 4 neighboring positions
    directions = [0, 1; 0, -1; -1, 0; 1, 0];
    
    for i = 1:size(directions, 1)
        newRow = row + directions(i, 1);
        newCol = col + directions(i, 2);
        if is_valid_cell(m, newRow, newCol, required)
            if m(newRow, newCol) == required
                r = newRow;
                c = newCol;
                min_value = required;
                break;
            end
        end
    end
    
    if r == -1 || c == -1
        min_value = inf;
    end
end

function is_valid = is_valid_cell(m, row, col, required)
    % Check if the cell is within the boundaries and has a valid value
    is_valid = (row > 0 && row <= size(m, 1) && col > 0 && col <= size(m, 2) && m(row, col) > 0);
end
