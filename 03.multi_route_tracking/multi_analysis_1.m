clc;clear;
%load('D:\code\multi_uav_test\2_N2T005_test2.mat')


load('D:\03.code\data_analysis\four_UAV_test\rectangle0904_2_uav1.mat');

load('D:\03.code\data_analysis\four_UAV_test\rectangle0904_2_uav2.mat');
load('D:\03.code\data_analysis\four_UAV_test\rectangle0904_2_uav3.mat');

n = length(X1);
h = animatedline;
axis([0 1000 0 1000])
axis equal
aviobj = VideoWriter('D:\03.code\data_analysis\four_UAV_test\rectangle0904_2.avi');
    open(aviobj);
for k = 1:9:n
    x1 = [];
    y1 = [];
    x2 = [];
    y2 = [];
    x3 = [];
    y3 = [];
    if k+3 > n
        x1 = X1(1,k:n);
        y1 = Y1(1,k:n);
        x2 = X2(1,k:n);
        y2 = Y2(1,k:n);
        x3 = X3(1,k:n);
        y3 = Y3(1,k:n);
    else
        x1 = X1(1,k:k+3);
        y1 = Y1(1,k:k+3);
        x2 = X2(1,k:k+3);
        y2 = Y2(1,k:k+3);
        x3 = X3(1,k:k+3);
        y3 = Y3(1,k:k+3);
    end
    %addpoints(h,X(k),Y(k)); 
    addpoints(h,x1,y1);  
    drawnow
    hold on;
    plot(x2,y2,'r');
    plot(x3,y3,'g');
    
    %plot(obs1_x(k),obs1_y(k),'or');
   % hold on;
   % plot(obs2_x(k), obs2_y(k),'or');
    frame = getframe(gcf);
    writeVideo(aviobj,frame);
end
close(aviobj);