function SL_TR_err()
clc;
clear;
load('D:\03.code\data_analysis\RT_straight_line\SL_N5T01.mat')
x1 = X;
y1 = Y;


xctl = 1:1:7950;
yctl = xctl * 3;




%% calculate the tracking error
mindis = [];
n = length(x1);
for i=1:n
    mindis(i) = abs(3*x1(i) - y1(i)) / sqrt(10);
end

total_err = 0;
for i = 1:1:n
    total_err = total_err + mindis(i);
end
mean_err = total_err / n;

save('D:\03.code\data_analysis\RT_straight_line\SL_N5T01_err.mat','mindis','mean_err');

figure(1)
plot(mindis);
grid on
end

