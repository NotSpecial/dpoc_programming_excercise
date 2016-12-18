main;

global timer J_debug;

fprintf('Time to compute: %d s\n', timer);

n_iter = size(J_debug, 2);

fprintf('Iteration steps: %d\n', n_iter);

figure();
plot([1:n_iter], J_debug);