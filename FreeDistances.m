function [ distances ] = FreeDistances( currState, targetCoordinates, map )
%FREEDISTANCES Distances from target states if view is free.
% 	Compute the distance for every state in targetStates if no tree or bush
% 	is obstructing the field of view. If the view is obstructed, the
% 	distance will be infinite.

n_targets = size(targetCoordinates, 1);
distances = Inf * ones(n_targets,1);

for i = 1:n_targets
    target = targetCoordinates(i,1:2);
   if freeView(currState, target, map)
        distances(i) = max(abs(currState - target));
   end
end

end

function [ free ] = freeView( currState, target, map )
%FREEVIEW returns true, iff x or y coordinates of P1(x,y) and P2(x,y) align
% and if there is no obstacle in line of sight between them
if any( currState - target == 0)
    % Aligned in n direction:
    if currState(1)==target(1)
        range_n = currState(1);
        if currState(2)>target(2)
            range_m = target(2)+1 : currState(2)-1;
        else
            range_m = currState(2)+1 : target(2)-1;
        end 
    end

    % Aligned in m direction:
    if currState(2)==target(2); %ycoord of points
        range_m = currState(2);
        if currState(1)>target(1)
            range_n = target(1)+1 : currState(1)-1;
        else
            range_n = currState(1)+1 : target(1)-1;
        end
    end

    free = ~any( map(range_m, range_n) > 0 );
else
    free = 0;
end
end
