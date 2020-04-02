
clear all;
clc;

step_delay = 100000;
step_delay_ = 100000;
rest = 0;
data = [];

n = -50

for l=1:100
    n=n+1; % Starts out negative and slowly goes positive.
    new_step_delay = int64(step_delay - ((2 * step_delay + rest) / (4 * n + 1)));
    new_step_delay_ = int64(step_delay_ - ((2 * step_delay_) / (4 * n + 1)));
    
    step_delay = new_step_delay;
    step_delay_ = new_step_delay_;
    
    rest = rem((2 * step_delay + rest),(4 * n + 1));
    data = [data; n new_step_delay new_step_delay_];
end
figure;
plot(data(:,1),data(:,2),data(:,1),data(:,3));