clc;
clear;
load('11.mat')
load('12.mat')
load('13.mat')
load('21.mat')
load('22.mat')
load('23.mat')
load('31.mat')
load('32.mat')
load('33.mat')
%%
p1x=0;p1y=0;
[p2x,p2y]=Convert(-124.3241,48.2669,-124.3179,48.268);
[p3x,p3y]=Convert(-124.3241,48.2669,-124.3151,48.2645);
[p4x,p4y]=Convert(-124.3241,48.2669,-124.3172,48.2609);
[p5x,p5y]=Convert(-124.3241,48.2669,-124.3245,48.2621);
figure(1)
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
  
%plot(xctlp,yctlp,'LineWidth',2,'LineStyle','--'); % 绘制控制多边形；
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
sp = spmak(knots,ctrpoint);
fnplt(sp);

plot(px11,py11,'LineWidth',1,'color','r');
hold on
grid on
plot(px21,py21,'LineWidth',1,'color','g');
plot(px31,py31,'LineWidth',1,'color','k');

%%
figure(2)
plot(s11,mindis11,'LineWidth',2,'color','r');
hold on;
grid on
plot(s21,mindis21,'LineWidth',2,'color','g');
plot(s31,mindis31,'LineWidth',2,'color','b');

figure(3)
plot(s12,mindis12,'LineWidth',2,'color','r');
hold on;
grid on
plot(s22,mindis22,'LineWidth',2,'color','g');
plot(s32,mindis32,'LineWidth',2,'color','b');

figure(4)
plot(s13,mindis13,'LineWidth',2,'color','r');
hold on;
grid on
plot(s23,mindis23,'LineWidth',2,'color','g');
plot(s33,mindis33,'LineWidth',2,'color','b');