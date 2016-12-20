% Init debug struct
global debug;
debug = struct();

main;

% Reload debug struct (main clears everything) and display it
global debug;
disp(debug);

figure();
plot(1: size(debug.J_val, 2), debug.J_val);
