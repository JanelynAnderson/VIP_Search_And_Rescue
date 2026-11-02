clc
Num_Segments = size(out.Snake_Simulated_q.Data, 2);

q_sim = cat(3, out.Snake_Simulated_q.Data,...
    out.Snake_Simulated_q.Time*ones(1, Num_Segments));

q_target = cat(3, out.Snake_Target_q.Data,...
    out.Snake_Target_q.Time*ones(1, Num_Segments));

% display
figure();
tiledlayout("flow");
for i = 1:Num_Segments
    ax = nexttile;
    hold on;
    d_sim = q_sim(:, i, :);
    d_target = q_target(:, i, :);
    plot(d_sim(:, 2), ((180/pi)*d_sim(:, 1)));
    plot(d_target(:, 2), ((180/pi)*d_target(:, 1)));
    xlabel("Time [s]");
    ylabel("Angle [deg]");
    title(ax, strcat("Segment ", string(i)));
end 