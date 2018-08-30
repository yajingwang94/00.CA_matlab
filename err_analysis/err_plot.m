
clc;
clear;

load('D:\03.code\data_analysis\RT_NT_analysis\N5T005_err.mat')
err0 = minDis1;
n = length(err0)-17;
mean_err0 = zeros(1,n);
for i = 1:1:n
    mean_err0(i) = mean_err;
end


load('D:\03.code\data_analysis\RT_s_forward\N5T005_err.mat')
err1 = minDis1;
n = length(err1)-14;
mean_err1 = zeros(1,n);
for i = 1:1:n
    mean_err1(i) = mean_err;
end







figure(1)
plot(err0);
hold on;
plot(err1);
hold on;
grid on;

legend('\fontsize{16} Vd = V = 19 m/s','\fontsize{16} Vd varies with the state') ;
xlabel('\fontsize{16}  t');
ylabel('\fontsize{16} y / m');

%figure(2)
%plot(mean_err0);
%hold on;
%plot(mean_err1);
%hold on;
%plot(mean_err2);
%hold on;
%plot(mean_err3);
%hold on;
%grid on;
%legend('\fontsize{16}N = 2, T = 0.05s','\fontsize{16} N = 5, T = 0.05s','\fontsize{16} N = 10, T = 0.05s', '\fontsize{16} N = 20, T = 0.05s') ;
%ylabel('\fontsize{16} y / m');

