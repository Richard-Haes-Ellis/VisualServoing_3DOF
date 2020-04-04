clear all;
clc;

a = 70000;
maxv = 30000;

data = [];

for x=1000:100:100000
    for t=5:5:60
        speed1 = (1/2)*a*t+(1/2)*a*sqrt(t*t-4*(x/a));
        speed2 = (1/2)*a*t-(1/2)*a*sqrt(t*t-4*(x/a));
        data = [data; x t speed1 speed2]
        pause()
    end
    x
end

scatter(data(:,1),data(:,2));