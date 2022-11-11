%% Generation of initial population
KP = 1;                         %Proportional gain
KI = 1;                         %Integral gain
p = sobolset(2);                %Generation of sobolset
x_parent = net(p,101);          %101 samples taken because the first value is (0,0) which will cause divition by zero in the optimizeControlSystem function
x_parent = x_parent(2:end,:);   % The first varible (0,0) is removed to form a sample size of 100

% The gain values are rescalled based on the information from the knowledge discovery
x_parent(:,1) = rescale(x_parent(:,1),0,0.6);
x_parent(:,2) = rescale(x_parent(:,2),0.15,0.45);

% Generation of initial design population
z_parent= optimizeControlSystem(x_parent);  % This function has modifications made to gain margin in order to maximise it

% Deciding the priority and goals
goal = [1 .14 60 2 10 10 10 20 1];
priority = [3 2 2 1 0 2 0 0 1];

%% NSGA-II algorithm
% Loop runs for 250 iterations or till convergence occurs
for i=1:250
    % Rank and crowding distance calculated
    rank = rank_prf(z_parent,goal,priority);
    dist = crowding(z_parent,rank);
    rank = max(rank) - rank;    % Rank inverted as selection function assumes higher values with better fitness
    fitness = [rank dist];
    % Selecting half the population based on fitness
    selection = btwr(fitness,round(height(z_parent)/2));
    selection(selection>height(z_parent)) = height(z_parent);     % This is to incorporate an error in the selection function
    % Polynomial mutation to parent design variables
    x_child = polymut(x_parent(selection,:), [0 1;0 1]);
    % Calculating new design solutions
    z_child= optimizeControlSystem(x_child);
    % Combining parent and child population
    z_combined = [z_parent;z_child];
    x_combined = [x_parent;x_child];
    % Calcuate fitness of new population
    rank = rank_prf(z_combined,goal,priority);
    dist = crowding(z_combined,rank);
    % Selecting the best solutions
    loc = reducerNSGA_II(z_combined,rank,dist,height(z_parent));
    % form the new parent population
    z_parent = z_combined(loc,:);
    x_parent = x_combined(loc,:);
    % Calculating the hypervolume of the design solution
    HV(i) = Hypervolume_MEX(z_parent);
    % Breaking the loop if convergence occurs
    if HV(i)==0
        break
    end
end
% Plot the hypervolume value for each iteration
plot(HV);
% Calculate the design solution using
z = optimizeControlSystem2(x_parent); % This is the original function with the actual formula to calculate the design solution