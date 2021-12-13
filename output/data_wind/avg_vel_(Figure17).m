%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M_0 = readmatrix('vel_data_single_agent.csv');
M_0 = readmatrix('vel_data_0.csv');
M_30 = readmatrix('vel_data_30.csv');
M_60 = readmatrix('vel_data_60.csv');
M_90 = readmatrix('vel_data_90.csv');
M_120 = readmatrix('vel_data_120.csv');
M_150 = readmatrix('vel_data_150.csv'); 
M_180 = readmatrix('vel_data_180.csv');

% plot data
hold on;

% p_0 =  plot(M_0);
p_0 =  plot(mean(M_0, 1));
p_30 =  plot(mean(M_30, 1));
p_60 =  plot(mean(M_60, 1));
p_90 =  plot(mean(M_90, 1));
p_120 =  plot(mean(M_120, 1));
p_150 =  plot(mean(M_150, 1));
p_180 =  plot(mean(M_180, 1));

xlabel('Number of simulation steps');
ylabel('Velocity');


legend([p_0(1), p_30(1), p_60(1), p_90(1), p_120(1), p_150(1), p_180(1)], ...
    '\alpha = 0°', '\alpha = 30°', '\alpha = 60°', '\alpha = 90°', '\alpha = 120°', '\alpha = 150°', '\alpha = 180°');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
