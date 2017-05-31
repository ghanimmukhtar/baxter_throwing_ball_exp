close all;
clear all;

for i=1:5,
    data{i} = load(['feedback.txt']);
end

for joint =1:7,
    figure(joint);
    hold on;
    for i=1:5,
        t = data{i}(2:end, 1) - data{i}(2, 1);
        plot(t, data{i}(2:end, joint+1), 'r');
        plot(t, data{i}(2:end, joint+1+7), 'b');
    end;
    legend('desired', 'real');
    xlabel('time (sec.)');
    ylabel('joint angle (rad.)');
    title(['joint ' int2str(joint)]);
end;

% alalysis
    % control error
    for i=1:5,
        error = data{i}(2:end, 2:8) - data{i}(2:end, 2+7:8+7);
        mean(abs(error)) * 180 / pi
        max(abs(error)) * 180 / pi
        min(abs(error)) * 180 / pi
    end;

%%

data{6} = load(['feedback_trj_4.txt']);
for joint =1:7,
    figure(joint);
    hold on;

    i=6;
    t = data{i}(2:end, 1) - data{i}(2, 1);
    plot(t, data{i}(2:end, joint+1), 'r');
    plot(t, data{i}(2:end, joint+1+7), 'b');

    legend('desired', 'real');
    xlabel('time (sec.)');
    ylabel('joint angle (rad.)');
    title(['joint ' int2str(joint)]);
    
end;    