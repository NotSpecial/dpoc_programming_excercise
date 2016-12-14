function [ distances ] = FreeDistances( currState, targetCoordinates, map )
%FREEDISTANCES Distances from target states if view is free.
% 	Compute the distance for every state in targetStates if no tree or bush
% 	is obstructing the field of view. If the view is obstructed, the
% 	distance will be infinite.
%
%   Input arguments:
%
%       currState:
%           A (1 x 2)-matrix of the state the others will be compared to.
%
%       targetCoordinates:
%           A (n x 2)-matrix where the n-th row contains the position of
%           the n-th element to get distance to sourceState. This would
%           most likely be either cameras or the mansion.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   Output arguments:
%
%       distances:
%           A (nx1)-matrix containing the distances of the target coordinates
%           to the current state

temp = size(targetCoordinates);
numberCameras = temp(1);
distances = Inf * ones(numberCameras,1);

state_x = currState(1);
state_y = currState(2);

for i = 1:numberCameras
    %check x-coordinate equivalence:
    if state_x == targetCoordinates(i,1)
        if freeView(currState,targetCoordinates(i,1:2),map)
            distances(i) = abs(state_y - targetCoordinates(i,2));
        end
    end
    
    %check y-coordinate equivalence:
    if state_y == targetCoordinates(i,2)
        if freeView(currState,targetCoordinates(i,1:2),map)
            distances(i) = abs(state_x - targetCoordinates(i,1));
        end
    end
end

function [ free ] = freeView( P1, P2, map )
%FREEVIEW returns true, iff x or y coordinates of P1(x,y) and P2(x,y) align
% and if there is no obstacle in line of sight between them
free=0;

%xalignment:
if P1(1)==P2(1) %xcoord of points
    free=1;
    if P1(2)>P2(2)
        range = [P2(2)+1 : P1(2)-1];
    else
       range = [P1(2)+1 : P2(2)-1];
    end
       
    for yi = range
        if map(yi,P1(1))>0
            free=0;
            break;
        end
    end
end
    
%yalignment:
if P1(2)==P2(2); %ycoord of points
    free=1;
    if P1(1)>P2(1)
        range = [P2(1)+1 : P1(1)-1];
    else
       range = [P1(1)+1 : P2(1)-1];
    end
    for xi = range
        if map(P1(2),xi)>0
            free=0;
            break;
        end
    end
end

end


end
