%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% import csv of distances with extended cohesion term added
Table_wo = readtable('avg_dist_wo.csv');

% import csv of distances without extended cohesion term added
Table_w = readtable('avg_dist_w.csv');

% convert to array
Array_wo = table2array(Table_wo);
Array_w = table2array(Table_w);

% calculate average of rows
% a entry of -1 corresponds to an agent not having any neighbors in that
% simulation step, so we don't consider him for the average at that
% simulation step

avg_wo = zeros(size(Array_wo,2), 1);
for i = 1:size(Array_wo,2)
    n = 0;
    for j = 1:size(Array_wo,1)
        if (Array_wo(j,i) >= 0)
            avg_wo(i) = avg_wo(i) + Array_wo(j,i);
            n = n+1;
        end
    end
    avg_wo(i) = avg_wo(i) / n;
end

avg_w = zeros(size(Array_w,2), 1);
for i = 1:size(Array_w,2)
    n = 0;
    for j = 1:size(Array_w,1)
        if (Array_w(j,i) >= 0)
            avg_w(i) = avg_w(i) + Array_w(j,i);
            n = n+1;
        end
    end
    avg_w(i) = avg_w(i) / n;
end

% plot data
hold on;

p_w = plot(avg_w);
p_wo = plot(avg_wo);

ylim([48 58]);
xlabel('Number of simulation steps');
ylabel('Euclidean distance');

legend([p_w(1), p_wo(1)], ...
    'with extended cohesion term', 'without extended cohesion term');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
