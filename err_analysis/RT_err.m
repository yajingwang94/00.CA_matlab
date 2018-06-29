function TR_err_1()
clc;
clear;
load('D:\03.code\data_analysis\RT_s_forward\N5T005_test.mat')

x1 = X;
y1 = Y;


%% points to pass through
ctlp=[-124.3241, -124.3178, -124.3150, -124.3171, -124.3244; 48.2669,   48.2681,   48.2646,   48.2610,   48.2622];
xctlp=[0,0,0,0,0];
yctlp=[0,0,0,0,0];
for i=2:5 
   [xctlp(i),yctlp(i)]=Convert(ctlp(1,1), ctlp(2,1), ctlp(1,i), ctlp(2,i));
end

%% pick out the necessary data
for i=1:length(x1)
    dis_start1(i) =(x1(i) - xctlp(1))^2 + (y1(i) - yctlp(1))^2;
    dis_end1(i)   =(x1(i) - xctlp(5))^2 + (y1(i) - yctlp(5))^2;
end
[mindis_start1,data_begin1]=min(dis_start1);
[mindis_end1,data_end1]=min(dis_end1);
data_end1 = data_end1 - 1;
t1 = data_end1 - data_begin1;
pos1 = [x1(data_begin1:data_end1);y1(data_begin1:data_end1)];



%figure(1)
%plot(x1(data_begin1:data_end1),y1(data_begin1:data_end1),'g','LineWidth',2);
%grid on
%hold on


%% generate the Bspline
knots=[0 0 0 0 0.2536775, 0.4842969, 0.7088977 1 1 1 1];
ctrpoint=[0, 44.47752, 288.4869, -243.5291, -852.8672, -522.6165, -522.6165; 0, 155.4366, 503.6388, 725.7311,  614.3641,  -22.20579, -22.20579];

%plot(xctlp,yctlp,'LineWidth',2,'LineStyle','--');
sp = spmak(knots,ctrpoint);
%fnplt(sp);

%% calculate the tracking error
minDis1 = getErr(pos1, t1, sp);
n = length(minDis1);
total_err = 0;
for i = 1:1:n
    total_err = total_err + minDis1(i);
end
mean_err = total_err / n;

save('D:\03.code\data_analysis\RT_s_forward\N5T005_test_err.mat','minDis1','mean_err');

figure(2)
plot(minDis1);
grid on
end

function [mindis] = getErr(pos, t, sp)
    mindis = [];
    deltas = 200;
    stotal = 0;
    for i = 2:t+1
        stotal = stotal + sqrt((pos(1,i) - pos(1,i-1))^2 + (pos(2,i) - pos(2,i-1))^2); %×Ü»¡³¤
    end
    s = 0:stotal/(t-1):stotal;
    for i = 1:t
        if s(i) + deltas<=stotal && s(i)-deltas>=0        
            sa = s(i)-deltas:1:s(i)+deltas;
        end
        if s(i)-deltas<0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            sa = 0:1:s(i)+deltas;
        end
        if s(i)+deltas>stotal
            sa = s(i)-deltas:1:stotal;
        end
        u = sa/stotal;
        csp = fnval(sp,u);
        for j = 1:length(csp)
            dis(j) = sqrt((csp(1,j)-pos(1,i))^2+(csp(2,j)-pos(2,i))^2);
        end
        [mindis(i),dex(i)]=min(dis);
    end
end