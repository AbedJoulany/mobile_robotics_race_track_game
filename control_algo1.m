function [ay, ax, index_along_path] = control_algo1(y, x, vy, vx, path_y, path_x, index_along_path, target_y, target_x)
    % control_algo1 controls the robot by looking n steps ahead.
    % This function assumes a continuous path from initial position to target.
    %
    % input:
    %  y, x - scalars, the current position of the robot.
    %  vy, vx - scalars, the current velocity of the robot.
    %  path_y, path_x - 1d arrays, the ordered positions of the path to follow.
    %  index_along_path - scalar, the current index along the path where the robot is on.
    %  target_y, target_x - 1d arrays, the positions of the target.
    %
    % output:
    %  ay, ax - scalars, the accelerations for the next step.
    %  index_along_path - scalar, the updated current index along the path.

    % Check if the target is reached
    if is_target_reached(x, y, target_x, target_y)
        ax = -vx;
        ay = -vy;
        index_along_path = find_closest_point(x, y, path_x, path_y);
        return;
    end

    [ax, ay] = calculate_next_acceleration(y, x, vy, vx, path_y, path_x, index_along_path, target_y, target_x);
    [new_x, new_y, new_vx, new_vy] = update_position(x, y, vx, vy, ax, ay);
    index_along_path = find_closest_point(new_x, new_y, path_x, path_y);
end

function target_reached = is_target_reached(x, y, target_x, target_y)
    % Check if the current position is close enough to the target position
    tolerance = 1; % You can adjust the tolerance value as needed
    target_reached = any(abs(x - target_x) < tolerance & abs(y - target_y) < tolerance);
end

function [ac_x, ac_y] = calculate_next_acceleration(y, x, vy, vx, path_y, path_x, index_along_path, target_y, target_x)
    % Calculate the next acceleration based on the path and current state
    acceleration_options = get_acceleration_options();

    max_steps = max(abs(vx), abs(vy)) + 1;
    dir_index = index_along_path;
    for step = 1:max_steps
        if is_special_case(y, x)
            disp("Special case encountered");
        end

        [next_x, next_y] = get_next_path_point(path_x, path_y, dir_index, x, y);
        
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

            if ~is_correct_direction(direction_y, direction_x, ay_temp, ax_temp)
                continue;
            end

            [ac_x, ac_y] = evaluate_acceleration_option(x, y, vx, vy, path_x, path_y, index_along_path, [ay_temp, ax_temp], stop_options, step);
            if ~isempty(ac_x)
                return;
            end
        end
    end

    [ac_x, ac_y] = default_acceleration();
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

    % Adjust direction if the step size is greater than 1
    if step > 1
        if abs(vx) > abs(vy)
            direction_x = -direction_x;
        elseif abs(vy) > abs(vx)
            direction_y = -direction_y;
        end
    end
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

function [ac_x, ac_y] = evaluate_acceleration_option(x, y, vx, vy, path_x, path_y, index_along_path, acceleration, stop_options, step)
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
        if can_stop_at_position(next_x, next_y, next_vx, next_vy, path_x, path_y, index_along_path, stop_options)
            [ac_x, ac_y] = deal(ax_temp, ay_temp);
            return;
        else
            disp("Can't stop at position");
        end
    end

    [ac_x, ac_y] = check_stop_options(x, y, vx, vy, path_x, path_y, stop_options, step);
end

function [ac_x, ac_y] = check_stop_options(x, y, vx, vy, path_x, path_y, stop_options, step)
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

function [new_x, new_y, new_vx, new_vy] = update_position(x, y, vx, vy, ax, ay)
    % Update the position based on current velocity and acceleration
    new_vx = vx + ax;
    new_vy = vy + ay;
    new_x = x + new_vx;
    new_y = y + new_vy;
end

function is_special = is_special_case(y, x)
    % Check if the current position is a special case
    is_special = (y == 81 && x == 30);
end

function is_on_path = is_on_path(x, y, path_x, path_y)
    % Check if the given position is on the path
    is_on_path = any((path_x == x) & (path_y == y));
end

function can_stop = can_stop_at_position(next_x, next_y, next_vx, next_vy, path_x, path_y, index_along_path, stop_options)
    % Check if we can stop and still be on the path
    can_stop = false;
    while next_vx ~= 0 || next_vy ~= 0
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
