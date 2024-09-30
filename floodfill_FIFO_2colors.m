function m = floodfill_FIFO_2colors(m, y1, x1, y2, x2, ColorToPaintOn, SeedColor1, SeedColor2, dC, slow, jump)
    % using a FIFO queue to implement flood fill (the breadth first search version).
    % paints an area in a matrix whose cells have the color ColorToPaintOn.
    % the painting starts from both from positions (y1,x1) and positions
    % (y2,x2). The flood progresses in two waves.
    % the first painting wave start with SeedColor1 and increases depending
    % on the distance from the (y1,x1) cells. Each step increases the color in
    % the amount dC.
    % the second painting wave start with SeedColor2 and increases depending
    % on the distance from the (y2,x2) cells. Each step increases the color in
    % the amount dC.
    % Using dC = 0 implements painting of all the pixels to SeedColor1 / SeedColor2.
    %
    % input:
    %  m
    %    matrix, the color map.
    % 
    %  y1, x1, y2, x2
    %    1D arrays, the starting points 1 and 2 to paint from.
    %
    %  ColorToPaintOn 
    %    scalar, only pixels with this color will be painted on.
    %
    %  SeedColor1, SeedColor2
    %    scalars, the colors to start painting with.
    %
    %  dC, 
    %    scalar, the delta to add to the color while painting,
    %    use dC = 0 to paint all the pixels to SeedColor.
    %    use dC = 1 to paint in color values as distance from the 
    %    starting points.
    %  
    %  slow, 
    %    boolean, if == 1, use presentation mode.
    % 
    %  jump
    %    scalar, how many steps to jump in slow mode.
    %    
    % output:
    %  m
    %    matrix, the updated map after painting.

    % initializing the queue for 3 values data ( (y,x,c) = location + color ) 
    lq = LQueue(100, 3); 
    
    % insert the starting point(s) for the first color
    for i = 1:numel(y1)
        lq = LQinsert(lq, [y1(i), x1(i), SeedColor1]);
    end
    
    % insert the starting point(s) for the second color
    for i = 1:numel(y2)
        lq = LQinsert(lq, [y2(i), x2(i), SeedColor2]);
    end
    
    slow_counter = 0;
    
    while ~LQisEmpty(lq)
        
        [lq, p] = LQgetPoint(lq); % FIFO, not LIFO, so implementing breadth first search
        
        y = p(1);
        x = p(2);
        c = p(3);
        
        % if x >= 2 && x <= mx - 1 && y >= 2 && y <= my - 1   % assuming all borders
        % are painted, so do not check for x,y values
        
        if m(y, x) == ColorToPaintOn
            m(y, x) = c;
            
            if slow
                slow_counter = slow_counter + 1;
                if slow_counter >= jump
                    slow_counter = 0;
                    imagesc(m);
                    % colormap(hot)
                    axis equal
                    axis off
                    drawnow
                end
            end
            
            % check which of the neighbors to insert to the queue 
            if m(y - 1, x) == ColorToPaintOn , lq = LQinsert(lq, [y - 1, x, c + dC]); end
            if m(y + 1, x) == ColorToPaintOn , lq = LQinsert(lq, [y + 1, x, c + dC]); end
            if m(y, x - 1) == ColorToPaintOn , lq = LQinsert(lq, [y, x - 1, c + dC]); end
            if m(y, x + 1) == ColorToPaintOn , lq = LQinsert(lq, [y, x + 1, c + dC]); end
            
        end
    end
    
    % end
    
end
