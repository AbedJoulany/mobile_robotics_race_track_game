function [ay, ax, index_along_path] = control_algo2(y, x, vy, vx, path_y, path_x, index_along_path, target_y, target_x)
    % control_algo2 controls the robot by allowing deviations from the path.
    % This function assumes a continuous path from initial position to target.
    %
    % input:
    %  y, x - scalars, the current position of the robot.
    %  vy, vx - scalars, the current velocity of the robot.
    %  path_y, path_x - 1d arrays, the ordered positions of the path to follow.
    %  index_along_path - scalar, the current index along the path where the robot is on.
    %  target_y, target_x - scalars, the positions of the target.
    %
    % output:
    %  ay, ax - scalars, the accelerations for the next step.
    %  index_along_path - scalar, the updated current index along the path.

    % Check if the target is reached


    % If not yet on the path, move towards the path (take off)
    if index_along_path == 1
        [ay, ax, index_along_path] = move_towards_path(y, x, vy, vx, path_y, path_x, index_along_path);
        return;
    end
    if is_on_path(x,y, target_x, target_y)
            index_along_target = find_closest_point(x,y,target_x, target_y);

        if index_along_target < length(target_x)
           ay = 0;
           ax = 0;
           if vy == 0
              ay = 1;
           end
           if vx ~=0
              ax = -vx;
           end

        else
            % Stop at the end of the path
            ay = -vy;
            ax = -vx;
        end
        return
    end

    [ax, ay] = calculate_next_acceleration(y, x, vy, vx, path_y, path_x, index_along_path, target_y, target_x);
    
    [new_x, new_y, new_vx, new_vy] = update_position(x, y, vx, vy, ax, ay);

    index_along_path = find_closest_point(new_x, new_y, path_x, path_y);
end

function [ac_x, ac_y] = calculate_next_acceleration(y, x, vy, vx, path_y, path_x, index_along_path, target_y, target_x)
    % Calculate the next acceleration based on the path and current state
    acceleration_options = get_acceleration_options();
    best_distance = -Inf; % Initialize the best distance as negative infinity
    best_ac_x = 0;
    best_ac_y = 0;

    max_steps = max(abs(vx), abs(vy)) + 3;
    dir_index = index_along_path;


    for step = 1:max_steps
        [next_x, next_y] = get_next_path_point(path_x, path_y, dir_index, x, y);
        if will_reach_target(x, y, target_x(1), target_y(1))
            next_x = target_x(1);
            next_y = target_y(1);
        end

        if isempty(next_x)
            ac_x = 0;
            ac_y = 0;
            return;
        end

        dir_index = dir_index + 1;
        [direction_x, direction_y] = calculate_direction(next_x, next_y, x, y, vx, vy, step);
        stop_options = calculate_stop_options(vx, vy);

        for k = 1:size(acceleration_options, 1)
            [ax_temp, ay_temp] = deal(acceleration_options(k, 2), acceleration_options(k, 1));

            if ~is_correct_direction(direction_y, direction_x, ay_temp+vy, ax_temp+vx)
                continue;
            end
            % x 1 y 1
            [temp_ac_x, temp_ac_y] = evaluate_acceleration_option(x, y, vx, vy, path_x, path_y, index_along_path, [ay_temp, ax_temp], stop_options, step, target_y, target_x);

            if ~isempty(temp_ac_x)
                % Calculate the distance the robot would travel with this acceleration
                [next_vx, next_vy] = deal(vx + temp_ac_x, vy + temp_ac_y);
                [next_x, next_y] = update_position(x, y, next_vx, next_vy, temp_ac_x, temp_ac_y);

                if is_within_deviation(next_x, next_y, path_x, path_y, 2)
                    distance_traveled = sqrt((next_x - x)^2 + (next_y - y)^2);

                    if distance_traveled > best_distance
                        best_distance = distance_traveled;
                        best_ac_x = temp_ac_x;
                        best_ac_y = temp_ac_y;
                    end
                end
            end
        end
    end

    if best_distance > -Inf
        ac_x = best_ac_x;
        ac_y = best_ac_y;
    else
        [ac_x, ac_y] = default_acceleration();
    end
end

function [ac_x, ac_y] = evaluate_acceleration_option(x, y, vx, vy, path_x, path_y, index_along_path, acceleration, stop_options, step, target_y, target_x)
    % Evaluate if the given acceleration option is valid
    [ax_temp, ay_temp] = deal(acceleration(2), acceleration(1));
    [next_vx, next_vy] = deal(vx + ax_temp, vy + ay_temp);

    if vx == 0 && vy == 0
        if is_on_path(x + ax_temp, y + ay_temp, path_x, path_y)
            [ac_x, ac_y] = deal(ax_temp, ay_temp);
            return;
        else


            [ac_x, ac_y] = deal([], []);
            return;
        end
    else
        [next_x, next_y] = deal(x + next_vx, y + next_vy);
    end

    if is_on_path(next_x, next_y, path_x, path_y)
        if can_stop_at_position(next_x, next_y, next_vx, next_vy, path_x, path_y)
            [ac_x, ac_y] = deal(ax_temp, ay_temp);
            return;

        end
    end

    [ac_x, ac_y] = evaluate_stop_options(x, y, vx, vy, path_x, path_y, step);
end

function [ac_x, ac_y] = evaluate_stop_options(x, y, vx, vy, path_x, path_y, step)
    stop_options = calculate_stop_options(vx, vy);

    % Check the stop options to find a valid acceleration
    for k = 1:size(stop_options, 1)
        [ax_temp, ay_temp] = deal(stop_options(k, 1), stop_options(k, 2));
        [next_vx, next_vy] = deal(vx + ax_temp, vy + ay_temp);

        if vx == 0 && vy == 0
            if is_on_path(x + ax_temp, y + ay_temp, path_x, path_y)
                [ac_x, ac_y] = deal(ax_temp, ay_temp);
                return;
            end
        else
            [next_x, next_y] = deal(x + next_vx, y + next_vy);
        end

        if is_on_path(next_x, next_y, path_x, path_y)
            [ac_x, ac_y] = deal(ax_temp, ay_temp);
            return;
        end
    end

    [ac_x, ac_y] = deal([], []);
end

function can_stop = can_stop_at_position(next_x, next_y, next_vx, next_vy, path_x, path_y)
    % Check if we can stop and still be on the path
    can_stop = false;
    while next_vx ~= 0 || next_vy ~= 0
        stop_options = calculate_stop_options(next_vx, next_vy);
        stop_found = false;
        for j = 1:size(stop_options, 1)
            [ax_temp, ay_temp] = deal(stop_options(j, 1), stop_options(j, 2));
            [temp_vx, temp_vy] = deal(next_vx + ax_temp, next_vy + ay_temp);
            [next_temp_x, next_temp_y] = deal(next_x + temp_vx, next_y + temp_vy);

            if is_on_path(next_temp_x, next_temp_y, path_x, path_y) 
                stop_found = true;
                break;
            end
        end
        if ~stop_found
            return;
        end

        [next_vx, next_vy] = deal(next_vx + ax_temp, next_vy + ay_temp);
        [next_x, next_y] = deal(next_x + temp_vx, next_y + temp_vy);
    end
    can_stop = true;
end


function acceleration_options = get_acceleration_options()
    % Return the possible acceleration options
    acceleration_options = [-1, 1; 0, 1; 1, -1; 1, 0; 1, 1; -1, -1; -1, 0; 0, -1; 0, 0];
end

function [next_x, next_y] = get_next_path_point(path_x, path_y, index_along_path, x, y)
    if index_along_path >= length(path_x)
        [next_x, next_y] = deal([], []);
        return;
    end
    % Get the next point on the path
    next_x = path_x(index_along_path + 1);
    next_y = path_y(index_along_path + 1);
end

function [direction_x, direction_y] = calculate_direction(next_x, next_y, x, y, vx, vy, step)
    % Calculate the direction to the next point, adjusting based on velocity and step size
    direction_x = sign(next_x - x);
    direction_y = sign(next_y - y);
end



function stop_options = calculate_stop_options(vx, vy)
    % Initialize stop options array
    stop_options = [];

    % Add stop options based on vx
    if vx > 0
        stop_options = [stop_options; [-1, -1]; [-1, 1]; [-1, 0]];
    elseif vx < 0
        stop_options = [stop_options; [1, -1]; [1, 1]; [1, 0]];
    end

    % Add stop options based on vy
    if vy > 0
        stop_options = [stop_options; [-1, -1]; [1, -1]; [0, -1]];
    elseif vy < 0
        stop_options = [stop_options; [-1, 1]; [1, 1]; [0, 1]];
    end

    % Remove options that increase the velocity in the wrong direction
    if vx > 0
        stop_options(stop_options(:, 1) > 0, :) = [];
    elseif vx < 0
        stop_options(stop_options(:, 1) < 0, :) = [];
    end

    if vy > 0
        stop_options(stop_options(:, 2) > 0, :) = [];
    elseif vy < 0
        stop_options(stop_options(:, 2) < 0, :) = [];
    end

    % Remove duplicate rows
    stop_options = unique(stop_options, 'rows');
end


function [new_x, new_y, new_vx, new_vy] = update_position(x, y, vx, vy, ax, ay)
    % Update the position based on current velocity and acceleration
    new_vx = vx + ax;
    new_vy = vy + ay;
    new_x = x + new_vx;
    new_y = y + new_vy;
end


function [ac_x, ac_y] = default_acceleration()
    % Return default acceleration values
    ac_x = 0;
    ac_y = 0;
end

function index = find_closest_point(x, y, path_x, path_y)
    % Find the closest point on the path to the given position
    distances = sqrt((path_x - x).^2 + (path_y - y).^2);
    [~, index] = min(distances);
end

function is_correct = is_correct_direction(direction_y, direction_x, ay_temp, ax_temp)
    % Check if the acceleration option is in the correct direction
    is_correct = (direction_y == sign(ay_temp)) && (direction_x == sign(ax_temp));
end



function is_within_deviation = is_within_deviation(x, y, path_x, path_y, max_deviation)
    max_deviation = 3;
    % Check if the given position is within the allowed deviation
    distances = sqrt((path_x - x).^2 + (path_y - y).^2);
    is_within_deviation = any(distances <= max_deviation);
end


function is_on_path = is_on_path(x, y, path_x, path_y)
    % Check if the given position is on the path
    is_on_path = any((path_x == x) & (path_y == y));
end



function [ay, ax, index_along_path] = move_towards_path(y, x, vy, vx, path_y, path_x, index_along_path)
    % Find the nearest point on the path
    [min_dist, min_idx] = min(sqrt((path_y - y).^2 + (path_x - x).^2));
    next_y = path_y(min_idx);
    next_x = path_x(min_idx);

    % Calculate the required acceleration to move towards the nearest path point
    ay = next_y - y;
    ax = next_x - x;

    % Ensure the acceleration does not exceed the allowed limit
    ay = max(min(ay, 1), -1);
    ax = max(min(ax, 1), -1);

    % Predict next position
    next_vy = vy + ay;
    next_vx = vx + ax;
    next_y_pos = y + next_vy;
    next_x_pos = x + next_vx;

    % Ensure the velocity does not cause the robot to exceed the path point
    if abs(next_y_pos - next_y) < abs(vy)
        ay = -vy; % Decelerate to stop at the path point
    end

    if abs(next_x_pos - next_x) < abs(vx)
        ax = -vx; % Decelerate to stop at the path point
    end

    % Update index if the robot is close enough to the path
    if min_dist < 1
        index_along_path = min_idx;
    end

    % Stop the robot if it's close enough to the target point
    if min_dist < 0.1
        ay = -vy;
        ax = -vx;
    end
end


function will_reach = will_reach_target(next_x, next_y, target_x, target_y)
    threshold = 5; % You can adjust this threshold as needed
    will_reach = sqrt((next_x - target_x)^2 + (next_y - target_y)^2) < threshold;
end