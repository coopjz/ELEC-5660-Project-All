function Optimal_path = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);
    
    
    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    MIN_X=0;
    MIN_Y=0;
    MIN_Z=0;

    GRID_RESOLUTION = 1;
    inflation_radius = 0;  
    inflation_cells = round(inflation_radius / GRID_RESOLUTION);

    block_factor = ceil(1/GRID_RESOLUTION);
    function [ind_x,ind_y,ind_z] = positionToIndex(Pos_x,Pos_y,Pos_z)
        ind_x = floor((Pos_x-MIN_X)/GRID_RESOLUTION)+1;
        ind_y = floor((Pos_y-MIN_Y)/GRID_RESOLUTION)+1;
        ind_z = floor((Pos_z-MIN_Z)/GRID_RESOLUTION)+1;
    end
    
    function [Pos_x, Pos_y, Pos_z] = indexToPosition(ind_x, ind_y, ind_z)
        Pos_x = (ind_x - 1) * GRID_RESOLUTION + MIN_X;
        Pos_y = (ind_y - 1) * GRID_RESOLUTION + MIN_Y;
        Pos_z = (ind_z - 1) * GRID_RESOLUTION + MIN_Z;
    end
    
    x_coord_num = floor(MAX_X *block_factor);
    y_coord_num = floor(MAX_Y *block_factor);
    z_coord_num = floor(MAX_Z *block_factor);
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));

    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1;
    

    %search map
    SEARCH_MAP=2*(ones(x_coord_num,y_coord_num,z_coord_num));
    
    % construct search map base on MAP, obstacle size is 1x1x1, start point
    % and end point are start -0.5, end -0.5
    for i = 1:MAX_X
        for j = 1:MAX_Y
            for k = 1:MAX_Z
                % Determine the corresponding indices in SEARCH_MAP
                x_start_idx = (i-1)*block_factor + 1;
                x_end_idx   = i*block_factor;
                y_start_idx = (j-1)*block_factor + 1;
                y_end_idx   = j*block_factor;
                z_start_idx = (k-1)*block_factor + 1;
                z_end_idx   = k*block_factor;
                if MAP(i,j,k) == -1
                    % For an obstacle, compute the inflated region boundaries.
                    x_inf_start = max(1, x_start_idx - inflation_cells);
                    x_inf_end   = min(x_coord_num, x_end_idx + inflation_cells);
                    y_inf_start = max(1, y_start_idx - inflation_cells);
                    y_inf_end   = min(y_coord_num, y_end_idx + inflation_cells);
                    z_inf_start = max(1, z_start_idx - inflation_cells);
                    z_inf_end   = min(z_coord_num, z_end_idx + inflation_cells);
                    
                    SEARCH_MAP(x_inf_start:x_inf_end, y_inf_start:y_inf_end, z_inf_start:z_inf_end) = -1;
                else
                    % For free, start, or target cells, just fill the block.
                    SEARCH_MAP(x_start_idx:x_end_idx, y_start_idx:y_end_idx, z_start_idx:z_end_idx) = MAP(i,j,k);
                end
            end
        end
    end
  
    [s_x, s_y, s_z] = positionToIndex(map(1,1)-0.5, map(1,2)-0.5, map(1,3)-0.5);
    [t_x, t_y, t_z] = positionToIndex(map(size_map,1)-0.5, map(size_map,2)-0.5, map(size_map,3)-0.5);
    
    SEARCH_MAP(s_x, s_y, s_z) = 1;
    SEARCH_MAP(t_x, t_y, t_z) = 0;

    % Main structure in the A* search =====================================================
    search_max_x = x_coord_num;
    search_max_y = y_coord_num;
    search_max_z = z_coord_num;

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    
    queue = [];  

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(search_max_x, search_max_y, search_max_z);

    % Arrays recording whether a node is expanded (popped from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(search_max_x, search_max_y, search_max_z);

    % Arrays recording the parent of each node
    parents = zeros(search_max_x, search_max_y, search_max_z, 3);
    
    
    %Start your code here ==================================================================
    % TODO
    movement =[1,0,0;
              -1,0,0;
               0,1,0;
               0,-1,0;
               0,0,1;
              0,0,-1;];
    
    all_movement=[];
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if ~(dx==0 && dy==0 && dz==0)
                    all_movement = [all_movement; dx, dy, dz];
                end
            end
        end
    end
%     movement = all_movement;
     h = @(x,y,z) 1*(abs(x-t_x) +abs(y-t_y) + abs(z-t_z));


    %h = @(x,y,z) sqrt((x - t_x)^2 + (y - t_y)^2 + (z - t_z)^2);


    g(s_x, s_y, s_z) = 0;
    f_start = g(s_x, s_y, s_z) + h(s_x, s_y, s_z);
    queue = [s_x, s_y, s_z, f_start];
    found = false;
    %% loop
    while ~isempty(queue)
        [~, idx] = min(queue(:,4));
        cur_node = queue(idx,:);
        queue(idx,:) =[];
        
        cx = cur_node(1);
        cy = cur_node(2);
        cz = cur_node(3);
        if cx == t_x && cy == t_y && cz == t_z
            found =true;
            break;
        end
        if expanded(cx, cy, cz)
            continue;
        end
         
        expanded(cx, cy, cz) = 1;
        
        for i = 1:size(movement,1)
            dx = movement(i,1);
            dy = movement(i,2);
            dz = movement(i,3);
            nx = cx + dx;
            ny = cy + dy;
            nz = cz + dz;

            if nx < 1 || nx > search_max_x || ny < 1 || ny > search_max_y || nz < 1 || nz > search_max_z
                continue;
            end

            if SEARCH_MAP(nx, ny, nz) == -1 || expanded(nx, ny, nz) == 1
                continue;
            end

            tentative_g = g(cx,cy,cz) + 1;

            if tentative_g < g(nx,ny,nz)
                parents(nx,ny,nz,:) =[cx,cy,cz];
                g(nx,ny,nz) =tentative_g;

                f = tentative_g + h(nx,ny,nz);

                queue = [queue; nx, ny, nz, f];
            end
        end
    end
    if found
        path = [t_x, t_y, t_z];
        cur_node = [t_x, t_y, t_z];
        while ~isequal(cur_node, [s_x, s_y, s_z])
            parent_node = squeeze(parents(cur_node(1), cur_node(2), cur_node(3), :))';
            path = [parent_node; path];
            cur_node = parent_node;
        end
        Optimal_path = path;
        for i = 1:size(Optimal_path,1)
            [px, py, pz] = indexToPosition(Optimal_path(i,1), Optimal_path(i,2), Optimal_path(i,3));
            Optimal_path(i,:) = [px, py, pz];
        end
        Optimal_path =Optimal_path + 0.5 * ones(size(Optimal_path));
    else
    disp('No valid path found from start to target.');
    Optimal_path=[];
    end
end


