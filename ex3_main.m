function ex3_main
   % ex3 robotics - basic framework for the solution by Yoram Yekutieli
   % Hadassah college
   % yoramye@hac.ac.il
   
   % NAME OF STUDENT:
   % DATE:  
   
   % In this exercise you will program a simulated robot to win a race:
   % move as fast as possible from start to finish in a track.
   
   % The track is given as free space in a map (matrix of walls cells / free cells.)
   % The track is a circular (but not convex) space and it width is more than 4 cells across.
   
   % The robot is defined by its properties:
   % a. Robot size is as a single cell in the map.
   % b. Robot movements are discrete and constrained to the free cells of the map.
   % c. Simulated time is discrete. The minimal time step is a single step of
   %    the main loop of the simulation.
   % d. Robot acceleration and speed are discrete. Maximal acceleration
   %    (deceleration) == 1 (-1).
   
  
   %% constants for the speed of painting in slow mode
   SLOW_1_FAST_0  = 0;   % 1 to view in slow mode, 0 to view in fast mode
   PAINT_JUMP     = 200;
   LINE_JUMP      = 10;
   
   %% A. SHOW THE MAP
   
   close all
   figure(1); clf; colormap gray
   
   temp_var = load('map.mat');
   map      = temp_var.map;
   [sy,sx]  = size(map);
   clear temp_var
   
   % colors to used in preparing the map
   % WALL_COLOR      = 1;
   % FREE_CELL_COLOR = 0;
   
   
   % show the map
   imagesc(map); axis equal; axis tight; axis off
   drawnow
   
   % maximize the figure
   set(gcf,'units','normalized','outerposition',[0 0 1 1]);
   % or use this:
   %    set(gcf,'Position',get(0,'ScreenSize'))
   
   if SLOW_1_FAST_0, pause; end
   
   
   %% B. paint two regions: the inner island, and the outer region
   WALL_COLOR      = 1;
   FREE_CELL_COLOR = 0;
   
   % constants for the speed of painting in slow mode
   jump           = PAINT_JUMP;        % x (jump) speed up in slow mode
   
   % --- paint the island from the middle ----
   % colors
   ISLAND_COLOR   = WALL_COLOR + 1;
   ColorToPaintOn = FREE_CELL_COLOR;   % paint on FREE_CELL_COLOR, stop on other colors
   SeedColor      = ISLAND_COLOR;      % start painting using this color
   dC             = 0;                 % do not change painting color while painting
   
   m = floodfill_FIFO(double(map),round(sy/2),round(sx/2),...
      ColorToPaintOn,SeedColor,dC,SLOW_1_FAST_0,jump);
   
   imagesc(m); axis equal; axis tight; axis off
   if SLOW_1_FAST_0, pause; end
   
   % --- paint the outer area from a point on the top left ----
   OUTER_COLOR    = WALL_COLOR + 2;
   SeedColor      = OUTER_COLOR;       % start painting with this color
   
   m = floodfill_FIFO...
      (double(m),4,4,ColorToPaintOn,SeedColor,dC,SLOW_1_FAST_0,jump);
   
   imagesc(m); axis equal; axis tight; axis off
   if SLOW_1_FAST_0, pause; end
   
   % change colors for next step
   max_color = 2*(sx+sy); % should be maximal for the current track
   
   % all wall pixels to color max_color, so it will be the maximal value,
   % thus not part of the possible path solution
   m(m==WALL_COLOR) = max_color;
   
   % all island pixels to color max_color*0.9
   m(m==ISLAND_COLOR) = round(max_color*0.9);
   
   % all outer area pixels to color max_color*0.9
   m(m==OUTER_COLOR) = round(max_color*0.9);
   
   % find the color of the track (should be FREE_CELL_COLOR == 0)
   TRACK_COLOR = m(102,38);
   
   % make sure all track pixels have FREE_CELL_COLOR (==0)
   m(m==TRACK_COLOR) = FREE_CELL_COLOR;
   
   imagesc(m); axis equal; axis tight; axis off
   colormap jet
   
   % save the map for later use
   map1 = m;
   
   if SLOW_1_FAST_0, pause; end
   
   %% C. find shortest path, first stage
   % --- paint the track area from the finish line ----
   
   % the finish line (as points)
   y_line = [102,103,104,105,106,107];
   x_line = [ 38, 38, 38, 38, 38, 38];
   
   ColorToPaintOn = FREE_CELL_COLOR;   % paint on FREE_CELL_COLOR, stop on other colors
   SeedColor      = 1; % start painting with color 1
   dC             = 1; % increase color with distance from the finish line
   
   m = floodfill_FIFO...
      (double(m),y_line,x_line,ColorToPaintOn,SeedColor,dC,SLOW_1_FAST_0,jump);
   
   imagesc(m); axis equal; axis tight; axis off;  colormap jet
   if SLOW_1_FAST_0, pause; end
   
   %% D. find shortest path, second stage
   % ----  gradient descent from start to finish line
   
   % start point
   start_y = 112;
   start_x = 47;
   
   PATH_COLOR   = max_color - 1;
   target_color = SeedColor;
   
   % 4 connected
   num_neighbors = 4;
   
   [path_y_4, path_x_4, m_4] = gradient_descent(m, start_y, start_x, ...
      target_color, PATH_COLOR, SLOW_1_FAST_0, LINE_JUMP, num_neighbors);
   
   path_y_4 = path_y_4(1:end-1);
   path_x_4 = path_x_4(1:end-1);
   
   imagesc(m_4); axis equal; axis tight; axis off; drawnow
   if SLOW_1_FAST_0, pause; end
   
   % 8 connected
   num_neighbors = 8;
   
   [path_y_8, path_x_8, m_8] = gradient_descent(m, start_y, start_x, ...
      target_color, PATH_COLOR, SLOW_1_FAST_0, LINE_JUMP, num_neighbors);

    % Remove trailing zeros from path_y_8
    last_nonzero_y = find(path_y_8, 1, 'last');
    path_y_8 = path_y_8(1:last_nonzero_y);

    % Remove trailing zeros from path_x_8
    last_nonzero_x = find(path_x_8, 1, 'last');
    path_x_8 = path_x_8(1:last_nonzero_x);
   
   imagesc(m_8); axis equal; axis tight; axis off; drawnow
   if SLOW_1_FAST_0, pause; end
   
   %% E. use the map (prior to the find path stage) to find the path furthest from the walls
   
   m1 = map1;
   
   % a reminder -  the colors are:
   % walls      ==  max_color
   % island     ==  max_color*0.9
   % outer area ==  max_color*0.9
   % track      ==  FREE_CELL_COLOR == 0
   
   % disconnect the walls from the connecting line:
   m1(94,48)  = FREE_CELL_COLOR;
   m1(95,48)  = FREE_CELL_COLOR;
   m1(115,38) = FREE_CELL_COLOR;
   
   imagesc(m1); axis equal; axis tight; axis off;  colormap jet
   if SLOW_1_FAST_0, pause; end
   
   %% F. trace the outlines
   % 1, find the pixels of the outer border of the island
   % use trace_line procedure to trace the line, starting from a given
   % initial point
   initial_y = 94;
   initial_x = 49;
   COLOR_TO_FOLLOW   = max_color;
   ISLAND_PATH_COLOR = round(max_color/2);
   
   jump           = LINE_JUMP;         % x (jump) speed up for the slow mode
   
   [island_line_y, island_line_x, m1] =...
      trace_line(m1, initial_y, initial_x,...
      COLOR_TO_FOLLOW, ISLAND_PATH_COLOR, SLOW_1_FAST_0, jump);
   
   % 2, find the inner pixels of the border of the outer area
   % use trace_line procedure to trace the line, starting from a given
   % initial point
   initial_y = 116;
   initial_x = 38;
   COLOR_TO_FOLLOW  = max_color;
   OUTER_PATH_COLOR = round(max_color/3);
   
   [outer_area_line_y, outer_area_line_x, m1] =...
      trace_line(m1, initial_y, initial_x,...
      COLOR_TO_FOLLOW, OUTER_PATH_COLOR,  SLOW_1_FAST_0, jump);
   
   
   % 3, put back the removed points that disconnected the walls
   m1(94,48)  = max_color;
   m1(95,48)  = max_color;
   m1(115,38) = max_color;
   
   imagesc(m1); axis equal; axis tight; axis off
   colormap jet
   
   if SLOW_1_FAST_0, pause; end
   
   %% G. painting inwards
   % paint in ISLAND_PATH_COLOR outwards from the island into the free
   % cells, and at the same time, paint in OUTER_PATH_COLOR inwards from the outer area
   % into the free cells.
   
   jump           = PAINT_JUMP*3;                % x (jump) speed up for the slow mode
   dC             = 0;
   ColorToPaintOn = FREE_CELL_COLOR;
   
   % before flooding, the lines should be painted to FREE_CELL_COLOR,
   % so the floodfill algorithm will be able to start with them
   for i=1:numel(island_line_y),
      m1(island_line_y(i),island_line_x(i)) = FREE_CELL_COLOR;
   end
   for i=1:numel(outer_area_line_y),
      m1(outer_area_line_y(i),outer_area_line_x(i)) = FREE_CELL_COLOR;
   end
   
   imagesc(m1); axis equal; axis tight; axis off; drawnow
   
   m2 = floodfill_FIFO_2colors(...
      m1, ...
      island_line_y,     island_line_x,    ...
      outer_area_line_y, outer_area_line_x, ...
      ColorToPaintOn,...
      ISLAND_PATH_COLOR,...
      OUTER_PATH_COLOR,...
      dC, SLOW_1_FAST_0, jump);
   
   % and after flooding, paint the lines back to max_color
   for i=1:numel(island_line_y),
      m2(island_line_y(i),island_line_x(i)) = max_color;
   end
   for i=1:numel(outer_area_line_y),
      m2(outer_area_line_y(i),outer_area_line_x(i)) = max_color;
   end
   
   imagesc(m2); axis equal; axis tight; axis off; drawnow
   if SLOW_1_FAST_0, pause; end
   
   %% H. trace the border
   % use trace_border to trace the border line between the two colors
   % ISLAND_PATH_COLOR & OUTER_PATH_COLOR
   
   initial_y  = 105;
   initial_x  = 45;
   initial_dy = 1;
   initial_dx = 1;
   jump       = LINE_JUMP;           % x (jump) speed up for the slow mode
   
   COLOR_1           = ISLAND_PATH_COLOR;
   COLOR_2           = OUTER_PATH_COLOR;
   BORDER_PATH_COLOR = round(max_color/1.3);
   
   [border_line_y, border_line_x, m3] =...
      trace_border(m2, initial_y, initial_x, initial_dy, initial_dx,...
      COLOR_1, COLOR_2, BORDER_PATH_COLOR,  SLOW_1_FAST_0, jump);
   
   imagesc(m3); axis equal; axis tight; axis off; drawnow
   if SLOW_1_FAST_0, pause; end
   
   %% I. movement!!
   
   % the map with colored regions is
   % map1
   
   m = map1;
   
   % the colors are:
   %  walls_color      =  max_color;
   %  island_color     =  max_color*0.9;
   %  outer_area_color =  max_color*0.9;
   track_color         =  FREE_CELL_COLOR; % == 0
   path_color          = round(max_color*0.3);
   initial_point_color = round(max_color*0.5);
   target_color        = round(max_color*0.2);
   
   allowed_max_color  = max(...
      [track_color,path_color,initial_point_color,target_color]);
   
   % start point
   y_initial = 112; 
   x_initial = 47;  
   
   % the finish line (as points)
   target_y = [102,103,104,105,106,107];
   target_x = [ 38, 38, 38, 38, 38, 38];
   
   % three run settings:
   % 1. shortest path with 4 connected neighbors, control algorithm 1
   % 2. shortest path with 8 connected neighbors, control algorithm 1
   % 3. furthest from the wall, control algorithm 2
   
   paths_y = {path_y_4, path_y_8, border_line_y};
   paths_x = {path_x_4, path_x_8, border_line_x};

   run_titles = {...
      'run 1: shortest path 4 connected, algo 1.',...
      'run 2: shortest path 8 connected, algo 1.',...
      'run 3: furthest from the walls, algo 2.'};
   
   control_algos = {@control_algo1, @control_algo1, @control_algo2};
   

   
   %% run three times:
   for run = 1:3
      
      m = map1;
      
      % the path to follow
      path_y = paths_y{run};
      path_x = paths_x{run};
      
      % for the visualization, add the path, tha initial point and the
      % target line to the map
      
      for i=1:numel(path_y)
         m(path_y(i),path_x(i)) = path_color;
      end
      
      m(y_initial,x_initial) = initial_point_color;
      
      for i=1:numel(y_line)
         m(target_y(i),target_x(i)) = target_color;
      end
           
      do_movement(m,...
         y_initial, x_initial,...
         target_y, target_x,...
         allowed_max_color,...
         path_y, path_x,...
         control_algos{run},...
         run_titles{run});
   end
  
   
   
end

%% ------------ local functions ------------

function do_movement(m, y_initial, x_initial, target_y, target_x,...
      allowed_max_color, path_y, path_x, control_algo, run_title)
   
   % show the initial map
   imagesc(m); axis equal; axis tight; axis off; title(run_title);
   hold on; plot(x_initial,y_initial,'ro');
   drawnow; pause

   
   % initial value for position variables
   y = y_initial;
   x = x_initial;
   
   % initial value velocity variables
   vy = 0;
   vx = 0;
   
   % initial value acceleration variables
   ay = 0; %#ok<NASGU>
   ax = 0; %#ok<NASGU>
   
   finished  = false;
   collided  = false;
   
   step_counter = 0;
   index_along_path = 1;
   
   while ~finished
      
      %       index_along_path
      
      % find the next acceleration
      [ay, ax, index_along_path] = control_algo(...
         y, x, vy, vx, path_y, path_x, index_along_path,...
         target_y, target_x);
      
      % make sure acceleration is not too big
      if abs(ay) > 1, ay = ay / abs(ay); end
      if abs(ax) > 1, ax = ax / abs(ax); end
      
      % calc the next velocity
      vy = vy + ay;
      vx = vx + ax;
      
      % calc the next position
      y = y + vy;
      x = x + vx;
      
      % check for collisions
      if m(y,x) > allowed_max_color
         finished = true;
         collided = true;
      end
      
      % check for finish line
      % is the current point on the finish line?
      if any ((target_y == y) & (target_x == x))
         % is the current velocity zero?
         if vy ==0 && vx == 0
            finished = true;
            collided = false;
         end
      end
      
      step_counter = step_counter + 1;
      
      title([run_title '      step = ' num2str(step_counter) ]);
      hold on; plot(x,y,'ro');
      drawnow
      pause(0.01);
      
   end
   
   if finished && ~collided
      title([run_title '      success in ' num2str(step_counter) ' steps.']);
   else
      title([run_title '      failure in ' num2str(step_counter) ' steps.']);
   end
   
   pause
   
end

