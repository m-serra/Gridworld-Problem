function [optimal_policy]=find_the_optimal_policy(discount, livingReward, noise )

    %value_func represents the map and the value of each state. 999 is obstacle.                    
    value_func = [0 0 0 0 0; 0 999 0 0 0; 0 999 1 999 10; 0 0 0 0 0; -10 -10 -10 -10 -10];
    
    %find the dimensions of the map
    [height,width] = size(value_func);
    
    %matrix with the 4 neighbours of each state. Index 1 indicates the
    %"name" of the state (1 through 12), index 2 indicates the neighbour (1
    %through 4).
    %
    %ex: surr(2,1) => state 2, north neighbour; surr(2,2) => state 2, east neighbour;
    %    surr(2,3) => state 2, south neighbour; surr(2,4) => state 2, west

    surr = repmat(999, [width*height 4]);
    
    %cue of states to aply value iteration initialized with -1
    cue = repmat(-1, [1 height*width]);
    
    %find the maximum reward => state where value iteration begins
    max_reward = 0;
    for i = 1:height
        for j = 1:width
            if (value_func(i,j) >= max_reward) && (value_func(i,j) ~= 999)
                init_state_x = j;
                init_state_y = i;
                max_reward = value_func(i,j);
            end
        end
    end
    
    %find tall the exit states and middle walls
    k = 1;
    for i = 1:height
        for j = 1:width
            if value_func(i,j) ~= 0
               exits_walls(k) = get_state_by_coord(j, i, width);
               k = k + 1;
            end
        end
    end
    
    %determine the initial surroundings of each state from the value
    %function
    for j = 1:height
        for i = 1:width %iterates all the states  
            
           state = get_state_by_coord(i,j,width);
 
            %up
            if(j-1 >= 1) surr(state,1) = value_func(j-1,i);
            else         surr(state,1) = 999;            %it's out of bounds
            end

            %right
            if(i+1 <= width) surr(state,2) = value_func(j,i+1);
            else         surr(state,2) = 999;
            end

            %down
            if(j+1 <= height) surr(state,3) = value_func(j+1,i);
            else         surr(state,3) = 999;
            end

            %left
            if(i-1 >= 1) surr(state,4) = value_func(j,i-1);
            else         surr(state,4) = 999;
            end           
        end
    end
     
    init_state = get_state_by_coord(init_state_x, init_state_y, width);
    index = 1;
    [cue, index] = add_state_to_cue(init_state, cue, index);
    
    present_state = init_state;
    aux_cue = cue;
    aux_value_func = value_func;
    j = 1;
      
    %number of iterations
    for i = 1:1000    
        %iterates initial_states
        while(cue(j) ~= -1)
            %find the states with value 0 arround the initial state
            for k = 1:4                      
               if (surr(present_state, k) ~= 999) && (surr(present_state, k) ~= -10) && (surr(present_state, k) ~= 1) && (surr(present_state, k) ~= 10)
                             
                    if      k == 1 state = present_state - width; %up    
                    elseif  k == 2 state = present_state +1; %right       
                    elseif  k == 3 state = present_state + width; %down   
                    elseif  k == 4 state = present_state -1; %left       
                    end         
                
                    [aux_cue, index] = add_state_to_cue(state, aux_cue, index);
                    [x,y] = get_coord_by_state(state, width);
                    [aux_value_func(y,x), surr] = Value_Iteration(state, value_func, surr, discount, livingReward, noise);
                    
               end
            end
            j = j + 1;
            present_state = aux_cue(j);
        end
        present_state = init_state;
        j = 1;
        cue = aux_cue;
        value_func = aux_value_func;
               
    end
    value_func
    policy = compute_policy(surr, height, width, exits_walls)
end


function [new_val, surr] = Value_Iteration(state, V, surr, discount, livingReward, noise)

    p_correct = 1 - noise;
    p_incorrect = noise / 2;
    reward = -999;
 
    %find the dimensions of the map
    [height,width] = size(V);
    
    [x,y] = get_coord_by_state(state, width);
    
    %find the action that maximizes the reward  ------------- pode haver mais do que uma                                             
    for k = 1:4
        if (surr(state, k) >= reward) && (surr(state, k) ~= 999)
            reward = surr(state, k);
            direction = k;
        end
    end
             
    %going up or down
    if direction == 1 || direction == 3  
        
        if      direction == 1 Vnext1 = V(y-1,x);
        elseif  direction == 3 Vnext1 = V(y+1,x);
        end
            
        if(x+1 > width || V(y,x+1) == 999)  Vnext2 = V(y,x);
        else Vnext2 = V(y,x+1);
        end
        
        if(x-1 < 1 || V(y,x-1) == 999)  Vnext3 = V(y,x);
        else Vnext3 = V(y,x-1); 
        end
        
        %Bellman
        new_val =  p_correct * discount * Vnext1 + p_incorrect * discount * Vnext2 + p_incorrect * discount * Vnext3;
        surr = refresh_surroundings(new_val, surr, state, width, height);
        
    %going right or left
    elseif direction == 2 || direction == 4
        
        if      direction == 2 Vnext1 = V(y,x+1);
        elseif  direction == 4 Vnext1 = V(y,x-1);
        end
        
        if(y+1 > height || V(y+1,x) == 999)  Vnext2 = V(y,x);
        else Vnext2 = V(y+1,x); 
        end
        
        if(y-1 < 1 || V(y-1,x) == 999)  Vnext3 = V(y,x);
        else Vnext3 = V(y-1,x) ;
        end
        
        %Bellman
        new_val = p_correct * discount * Vnext1 + p_incorrect * discount * Vnext2 + p_incorrect * discount * Vnext3;
        surr = refresh_surroundings(new_val, surr, state, width, height);
    end
end


function state = get_state_by_coord(x, y, width)   
    state = (y-1) * width + x;    
end

function [x, y] = get_coord_by_state(k, width)  
    y = ceil(k / width);        
    x = k - width *(y-1);  
end

function [cue, index] = add_state_to_cue( state, cue, index )
  
    [~,dim] = size(cue);

    %check if the state is already in the cue
    for i = 1:dim
        if state == cue(i) return;
        end;
    end
    
    cue(index) = state;
    index = index + 1;
end

function surr = refresh_surroundings(new_value, surr, state, width, height, exits_walls)
    
    %refresh northern neighbour surroundings
    if state - width >= 1
        surr(state - width, 3) = new_value;
    end
    
    %refresh eaestern beighbour surroundings
    if state + 1 <= width 
        surr(state + 1, 4) = new_value;
    end
    
    %refresh southern neighbour surroundings
    if state + width <= width * height
        surr(state + width, 1) = new_value;
    end
    
    %refresh western neighbour surroundings
    if state - 1 >= 1
        surr(state - 1, 2) = new_value;
    end   
end

function policy = compute_policy(surr, height, width, exits_walls)

    policy = repmat(0, [height width]);
    max_reward = -999;
    direction = -1;
    
    for i = 1:height*width %iterate states
        if(ismember(i, exits_walls)) 
            continue;
        end
        for j = 1:4 %iterate neighbours
            if(surr(i,j) >= max_reward && surr(i,j) ~= 999) 
                max_reward = surr(i,j);
                direction = j;
            end
        end
  
        [x, y] = get_coord_by_state(i, width);
        
        policy(y,x) = direction;
        max_reward = -999;
        direction = -1;   
    end
    
    %convert policy to professor's direction code (I used a different one
    %all along by mistake)
    for i = 1:height
        for j = 1:width
            if(policy(i,j) == 1)     policy(i,j) = 2;
            elseif(policy(i,j) == 2) policy(i,j) = 1;
            elseif(policy(i,j) == 3) policy(i,j) = 4;
            elseif(policy(i,j) == 4) policy(i,j) = 3;
            end
        end
    end
end

