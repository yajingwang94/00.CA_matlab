clc;
clear;
data1=importdata('20150827090215876_UAV1-21.txt',',',1);
len=length(data1.data(:,1));
lon=data1.data(:,2);
lat=data1.data(:,3);
for i=1:len
    [x(i),y(i)]=Convert(-124.3241,48.2669,lon(i),lat(i));
end
alt=data1.data(:,4);
pitch=data1.data(:,5);
roll=data1.data(:,6);
yaw=data1.data(:,7);
airspeed=data1.data(:,8);
g_speed=data1.data(:,9);
cyaw=data1.data(:,10);
cpitch=data1.data(:,11);
croll=data1.data(:,12);
tyaw=data1.data(:,13);
tpitch=data1.data(:,14);
troll=data1.data(:,15);
tthrott=data1.data(:,16);
rvx=data1.data(:,17);
rvy=data1.data(:,18);
rvz=data1.data(:,19);
accx=data1.data(:,20);
accy=data1.data(:,21);
accz=data1.data(:,22);
speN=data1.data(:,23);
speE=data1.data(:,24);
speD=data1.data(:,25);
gpsSta=data1.data(:,26);
nStNum=data1.data(:,27);
insSta=data1.data(:,28);

% data_begin=490;
% data_end=1155;
% 
% data_begin1=2750;
% data_end1=3480;
% t=data_end-data_begin;
% tt=0:50/(t):50;
%%
p1x=0;p1y=0;
[p2x,p2y]=Convert(-124.3241,48.2669,-124.3179,48.268);
[p3x,p3y]=Convert(-124.3241,48.2669,-124.3151,48.2645);
[p4x,p4y]=Convert(-124.3241,48.2669,-124.3172,48.2609);
[p5x,p5y]=Convert(-124.3241,48.2669,-124.3245,48.2621);
     

%%  %求取位置误差
%先求取到两个端点的最近的点的取值
for i=1:len
    dis_in(i)=(x(i)-p1x)^2+(y(i)-p1y)^2;
    dis_end(i)=(x(i)-p5x)^2+(y(i)-p5y)^2;
end
[dis_i,data_begin]=min(dis_in);
[dis_e,data_end]=min(dis_end);
 t=data_end-data_begin;
%%
figure(1)
plot(lon(data_begin:data_end),lat(data_begin:data_end),'g','LineWidth',2);
grid on
hold on

% plot(lon(data_begin1:data_end1),lat(data_begin1:data_end1),'r','LineWidth',2);
%      rectangle('Position',[113.22305,28.20731,0.0088,0.00417],...
%           'Curvature',[0,0],...
%          'LineWidth',2,'LineStyle','--')
    rectangle('Position',[-124.3242,48.2668,0.0002,0.0002],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
     rectangle('Position',[-124.3179,48.268,0.0002,0.0002],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
     rectangle('Position',[-124.3151,48.2645,0.0002,0.0002],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
     rectangle('Position',[-124.3172,48.2609,0.0002,0.0002],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
      rectangle('Position',[-124.3245,48.2621,0.0002,0.0002],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')

figure(2)
plot(x(data_begin:data_end),y(data_begin:data_end),'r');
px=x(data_begin:data_end);
py=y(data_begin:data_end);
grid on
hold on

  rectangle('Position',[p1x-10,p1y-10,20,20],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
     rectangle('Position',[p2x,p2y,20,20],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
     rectangle('Position',[p3x,p3y,20,20],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
     rectangle('Position',[p4x,p4y,20,20],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     hold on
      rectangle('Position',[p5x,p5y,20,20],...
          'Curvature',[1,1],...
         'LineWidth',2,'LineStyle','--')
     
     ctlp=[-124.3241,-124.3178,-124.3150,-124.3171,-124.3244;48.2669,48.2681,48.2646,48.2610,48.2622];
knots=[0 0 0 0 0.2536775, 0.4842969, 0.7088977 1 1 1 1];
ctrpoint=[0, 44.47752, 288.4869, -243.5291, -852.8672, -522.6165, -522.6165; 0, 155.4366, 503.6388,  725.7311, 614.3641, -22.20579, -22.20579];
xctlp=[0,0,0,0,0];
yctlp=[0,0,0,0,0];
for i=1:5 
   [xctlp(i),yctlp(i)]=Convert(ctlp(1,1), ctlp(2,1), ctlp(1,i), ctlp(2,i));
end
  


plot(xctlp,yctlp,'LineWidth',2,'LineStyle','--'); % 绘制控制多边形；
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
sp = spmak(knots,ctrpoint);
fnplt(sp);

%%
err=0;
ferr=0;
p=[x(data_begin:data_end);y(data_begin:data_end)];  %当前的位置
deltas=100;

stotal=0;
  for i=2:t+1
     stotal=stotal+sqrt((p(1,i)-p(1,i-1))^2+(p(2,i)-p(2,i-1))^2); %总弧长
  end
s=0:stotal/(t-1):stotal;

for i=1:t
    if s(i)+deltas<=stotal && s(i)-deltas>=0        
        sa=s(i)-deltas:1:s(i)+deltas;
    end
    if s(i)-deltas<0
    sa=0:1:s(i)+deltas;
    end
    if s(i)+deltas>stotal
        sa=s(i)-deltas:1:stotal;
    end

    u=sa/stotal;
    csp=fnval(sp,u);
    for j=1:length(csp)
        dis(j)=sqrt((csp(1,j)-p(1,i))^2+(csp(2,j)-p(2,i))^2);
    end
    [mindis(i),dex(i)]=min(dis);
    s(i)=u(dex(i));
    err=err+mindis(i)^2;
    ferr=ferr+abs(mindis(i));
end


%% %求取控制输入
con_in=cyaw(data_begin:data_end)-yaw(data_begin:data_end);
cin=0;
fcin=0;
for i=1:t
   cin=cin+con_in(i)^2;
   fcin=fcin+abs(con_in(i));
end
%%
% figure(4)
% subplot(3,1,1)
% plot(pitch(data_begin:data_end),'LineWidth',2,'LineStyle','-');
% xlim([0,t]);
% grid on
% subplot(3,1,2)
% plot(roll(data_begin:data_end),'LineWidth',2,'LineStyle','-');
% xlim([0,t]);
% grid on
% subplot(3,1,3)
% plot(yaw(data_begin:data_end),'LineWidth',2,'LineStyle','-');
% xlim([0,t]);
% grid on
% tp=length(con_in);
% 
% figure(5)
% plot(mindis);
% grid on
% 
% figure(6)
% plot(con_in);

