

steps=(1000*1000)/(2*25)

d = 1000000*0.676*sqrt(2/25)

sum = 0;
data = [];

for n=1:1:steps
    d = d - (2*d)/(4*n+1);
    sum = sum + d;
    data = [data; n sum];
end
  
subplot(3,1,1)
plot(data(:,1),1./data(:,2));
grid;

y = steps*sqrt(190.*data(:,1)+data(:,1));
subplot(3,1,2)
plot(data(:,1),data(:,2));
hold on
plot(data(:,1),y);
grid;
subplot(3,1,3);
plot(data(:,1),(abs(data(:,2)-y)));
grid;