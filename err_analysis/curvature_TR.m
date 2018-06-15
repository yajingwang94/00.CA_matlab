
clc;
clear;
load('D:\03.code\data_analysis\RT_NT_analysis\N5T005_err.mat')
err1 = minDis1(1,1:2000);



load('D:\03.code\data_analysis\SS_5_005.mat')
curvature = SSS(1,1:2000);

figure(1)
subplot(2,1,1)
plot(err1);
legend('\fontsize{16} tracking error, N = 5, T = 0.05');
xlabel('\fontsize{16}  t');
ylabel('\fontsize{16} y / m');
subplot(2,1,2)
plot(curvature);
hold on;
grid on;
legend('\fontsize{16} curvature of the spline');



