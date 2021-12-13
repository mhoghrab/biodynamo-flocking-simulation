%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = readmatrix('wind_deacc.csv');

figure;
p_1 = plot(M(2,:), M(1,:));

xlabel('Travelled distance');
ylabel('Velocity');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
p_2 = plot(M(1,:));

xlabel('Number of simulation steps');
ylabel('Velocity');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
