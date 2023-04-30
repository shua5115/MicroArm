% Goal: make a stack of six cylinders on the table

% The stack will be at a 45 degree angle from the robot, as to not
% interfere with the CV surface, but not be too far away.

% The first stack layer will start at 100 mm radius
% each cylinder is 30 mm in diameter, and will have a 5 mm gap in between
% But, the robot must place the furthest cylinders first.

layer1 = [170, 135, 100]
layer2 = [0, 0];
for i=1:(length(layer1)-1)
    layer2(i) = mean(layer1((i):(i+1)));
end
layer2
layer3 = mean(layer2)

% now to convert to vectors:
dir = [cosd(135);sind(135)];
xy = dir*[layer1, layer2, layer3]
z = [0, 0, 0, 31, 31, 62]
