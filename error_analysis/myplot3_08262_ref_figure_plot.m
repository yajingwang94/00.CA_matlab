%% plot the path of the UAV under winds
%% plot the error of the UAV under winds

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

lab=[11,12,13,21,22,23,31,32,33];
% lab=num2str(lab);
u_in=[cin11,cin12,cin13,cin21,cin22,cin23,cin31,cin32,cin33];
err_t=[err11,err12,err13,err21,err22,err23,err31,err32,err33];

fu_in=[fcin11,fcin12,fcin13,fcin21,fcin22,fcin23,fcin31,fcin32,fcin33];
ferr_t=[ferr11,ferr12,ferr13,ferr21,ferr22,ferr23,ferr31,ferr32,ferr33];
%      

ctlp=[-124.3242,-124.3179,-124.3151,-124.3172,-124.3245;48.2668,48.2680,48.2645,48.2609,48.2621];
knots=[0 0 0 0 0.2536775, 0.4842969, 0.7088977 1 1 1 1];
ctrpoint=[0, 44.47752, 288.4869, -243.5291, -852.8672, -522.6165, -522.6165; 0, 155.4366, 503.6388,  725.7311, 614.3641, -22.20579, -22.20579];
xctlp=[0,0,0,0,0];
yctlp=[0,0,0,0,0];
for i=2:5 
   [xctlp(i),yctlp(i)]=Convert(ctlp(1,1), ctlp(2,1), ctlp(1,i), ctlp(2,i));
end
  
sp = spmak(knots,ctrpoint);

slength=1986.5;
%%45o
figure(1)
plot(px11,py11,'b:','LineWidth',2);
hold on;
plot(px21,py21,'g-.','LineWidth',2);
hold on;
plot(px31,py31,'r--','LineWidth',2);
grid on
hold on


fnplt(sp,'k-');
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
plot(xctlp,yctlp,'k+','LineWidth',2,'MarkerSize',8);%画所有控制点

text(xctlp(1)+20,yctlp(1)-00,num2str(1),'fontsize',16,'color','k')
text(xctlp(2)-50,yctlp(2)-00,num2str(2),'fontsize',16,'color','k')
text(xctlp(3)-00,yctlp(3)-30,num2str(3),'fontsize',16,'color','k')
text(xctlp(4)+20,yctlp(4)-00,num2str(4),'fontsize',16,'color','k')
text(xctlp(5)-50,yctlp(5)+20,num2str(5),'fontsize',16,'color','k')

axis equal;

legend('\fontsize{16} wind speed: 2 kt','\fontsize{16} wind speed: 4 kt', '\fontsize{16} wind speed: 6 kt','\fontsize{16} ref. position') ;
xlabel('\fontsize{16}  x /m');
ylabel('\fontsize{16} y / m');

%%error
figure(2)
plot(s11*slength,mindis11,'b:','LineWidth',1);
hold on;
plot(s21*slength,mindis21,'g-.','LineWidth',1);
hold on;
plot(s31*slength,mindis31,'r--','LineWidth',1);
legend('\fontsize{16} wind speed: 2 kt','\fontsize{16} wind speed: 4 kt','\fontsize{16} wind speed: 6 kt');
 
merror11=mean(mindis11)  % mean 
stderror11=std(mindis11) % standard deviation
maxerror11=max(mindis11) % max 
varerror11=var(mindis11) % variance

merror21=mean(mindis21)  % mean 
stderror21=std(mindis21) % standard deviation
maxerror21=max(mindis21) % max 
varerror21=var(mindis21) % variance

merror31=mean(mindis31)  % mean 
stderror31=std(mindis31) % standard deviation
maxerror31=max(mindis31) % max 
varerror31=var(mindis31) % variance

% 90o
figure(3)
plot(px12,py12,'b:','LineWidth',2);
hold on;
plot(px22,py22,'g-.','LineWidth',2);
hold on;
plot(px32,py32,'r--','LineWidth',2);
grid on
hold on


fnplt(sp,'k-');
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
plot(xctlp,yctlp,'k+','LineWidth',2,'MarkerSize',8);%画所有控制点
text(xctlp(1)+20,yctlp(1)-00,num2str(1),'fontsize',16,'color','k')
text(xctlp(2)-50,yctlp(2)-00,num2str(2),'fontsize',16,'color','k')
text(xctlp(3)-00,yctlp(3)-30,num2str(3),'fontsize',16,'color','k')
text(xctlp(4)+20,yctlp(4)-00,num2str(4),'fontsize',16,'color','k')
text(xctlp(5)-50,yctlp(5)+20,num2str(5),'fontsize',16,'color','k')
axis equal;

legend('\fontsize{16} wind speed: 2 kt','\fontsize{16} wind speed: 4 kt', '\fontsize{16} wind speed: 6 kt','\fontsize{16} ref. position') ;
xlabel('\fontsize{16}  x /m');
ylabel('\fontsize{16} y / m');

figure(4)
plot(s12*slength,mindis12,'b:','LineWidth',1);
hold on;
plot(s22*slength,mindis22,'g-.','LineWidth',1);
hold on;
plot(s32*slength,mindis32,'r--','LineWidth',1);
legend('\fontsize{16} wind speed: 2 kt','\fontsize{16} wind speed: 4 kt','\fontsize{16} wind speed: 6 kt');

merror12=mean(mindis12)  % mean 
stderror12=std(mindis12) % standard deviation
maxerror12=max(mindis12) % max 
varerror12=var(mindis12) % variance

merror22=mean(mindis22)  % mean 
stderror22=std(mindis22) % standard deviation
maxerror22=max(mindis22) % max 
varerror22=var(mindis22) % variance

merror32=mean(mindis32)  % mean 
stderror32=std(mindis32) % standard deviation
maxerror32=max(mindis32) % max 
varerror32=var(mindis32) % variance


%% 135o
figure(5)
plot(px13,py13,'b:','LineWidth',2);
hold on;
plot(px23,py23,'g-.','LineWidth',2);
hold on;
plot(px33,py33,'r--','LineWidth',2);
grid on
hold on


fnplt(sp,'k-');
hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
plot(xctlp,yctlp,'k+','LineWidth',2,'MarkerSize',8);%画所有控制点
text(xctlp(1)+20,yctlp(1)-00,num2str(1),'fontsize',16,'color','k')
text(xctlp(2)-50,yctlp(2)-00,num2str(2),'fontsize',16,'color','k')
text(xctlp(3)-00,yctlp(3)-30,num2str(3),'fontsize',16,'color','k')
text(xctlp(4)+20,yctlp(4)-00,num2str(4),'fontsize',16,'color','k')
text(xctlp(5)-50,yctlp(5)+20,num2str(5),'fontsize',16,'color','k')
axis equal;

legend('\fontsize{16} wind speed: 2 kt','\fontsize{16} wind speed: 4 kt', '\fontsize{16} wind speed: 6 kt','\fontsize{16} ref. position') ;
xlabel('\fontsize{16}  x /m');
ylabel('\fontsize{16} y / m')

figure(6)
plot(s13*slength,mindis13,'b:','LineWidth',1);
hold on;
plot(s23*slength,mindis23,'g-.','LineWidth',1);
hold on;
plot(s33*slength,mindis33,'r--','LineWidth',1);
legend('\fontsize{16} wind speed: 2 kt','\fontsize{16} wind speed: 4 kt','\fontsize{16} wind speed: 6 kt');

merror13=mean(mindis13)  % mean 
stderror13=std(mindis13) % standard deviation
maxerror13=max(mindis13) % max 
varerror13=var(mindis13) % variance

merror23=mean(mindis23)  % mean 
stderror23=std(mindis23) % standard deviation
maxerror23=max(mindis23) % max 
varerror23=var(mindis23) % variance

merror33=mean(mindis33)  % mean 
stderror33=std(mindis33) % standard deviation
maxerror33=max(mindis33) % max 
varerror33=var(mindis33) % variance


% %speed 2kt
% figure(4)
% plot(px11,py11,'b:','LineWidth',2);
% hold on;
% plot(px12,py12,'g-.','LineWidth',2);
% hold on;
% plot(px13,py13,'r--','LineWidth',2);
% grid on
% hold on
% fnplt(sp,'k-');
% hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
% plot(xctlp,yctlp,'k+','LineWidth',2,'MarkerSize',8);%画所有控制点
% 
% axis equal;
% 
% %speed 4kt
% figure(5)
% plot(px21,py21,'b:','LineWidth',2);
% hold on;
% plot(px22,py22,'g-.','LineWidth',2);
% hold on;
% plot(px23,py23,'r--','LineWidth',2);
% grid on
% hold on
% fnplt(sp,'k-');
% hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
% plot(xctlp,yctlp,'k+','LineWidth',2,'MarkerSize',8);%画所有控制点
% 
% axis equal;
% 
% %speed 6kt
% figure(6)
% plot(px31,py31,'b:','LineWidth',2);
% hold on;
% plot(px32,py32,'g-.','LineWidth',2);
% hold on;
% plot(px33,py33,'r--','LineWidth',2);
% grid on
% hold on
% fnplt(sp,'k-');
% hold on; % 默认为hold off，此命令用来保留控制多边形的图形；
% plot(xctlp,yctlp,'k+','LineWidth',2,'MarkerSize',8);%画所有控制点
% axis equal;
% 
% 
% 

% 
% figure(9)
% plot(s13,mindis13,'r--','LineWidth',2);
% hold on;
% plot(s23,mindis23,'g--','LineWidth',2);
% hold on;
% plot(s33,mindis33,'b--','LineWidth',2);
% legend('\fontsize{16} wind speed: 2 kt, direction: 135^o','\fontsize{16} wind speed: 4 kt, direction: 135^o','\fontsize{16} wind speed: 6 kt, direction: 135^o');
% 
% % t22=0:tspan/(length(mindis22)-1):tspan;
% 
% % t23=0:tspan/(length(mindis23)-1):tspan;
% 
% % t31=0:tspan/(length(mindis31)-1):tspan;
% 
% % hold on;
% % t32=0:tspan/(length(mindis32)-1):tspan;
% 
% % hold on;
% % t33=0:tspan/(length(mindis33)-1):tspan;
% 
% 
% % legend('\fontsize{16} wind speed: 2 kt, direction: 45^o','\fontsize{16} wind speed: 2 kt, direction: 90^o', '\fontsize{16} wind speed: 2 kt, direction: 135^o',...
% %        '\fontsize{16} wind speed: 4 kt, direction: 45^o','\fontsize{16} wind speed: 4 kt, direction: 90^o', '\fontsize{16} wind speed: 4 kt, direction: 135^o',...
% %        '\fontsize{16} wind speed: 6 kt, direction: 45^o','\fontsize{16} wind speed: 6 kt, direction: 90^o', '\fontsize{16} wind speed: 6 kt, direction: 135^o') ;
%    
% % legend('\fontsize{16} wind speed: 2 kt, direction: 45^o','\fontsize{16} wind speed: 2 kt, direction: 90^o', '\fontsize{16} wind speed: 2 kt, direction: 135^o',...
% %        '\fontsize{16} wind speed: 4 kt, direction: 45^o','\fontsize{16} wind speed: 4 kt, direction: 90^o', '\fontsize{16} wind speed: 4 kt, direction: 135^o',...
% %        '\fontsize{16} wind speed: 6 kt, direction: 45^o','\fontsize{16} wind speed: 6 kt, direction: 90^o', '\fontsize{16} wind speed: 6 kt, direction: 135^o') ;
%    
% xlabel('\fontsize{16}  time /s');
% ylabel('\fontsize{16}  error / m')
% %  %%
% % % figure(4)
% % % subplot(3,1,1)
% % % plot(pitch(data_begin:data_end),'LineWidth',2,'LineStyle','-');
% % % xlim([0,t]);
% % % grid on
% % % subplot(3,1,2)
% % % plot(roll(data_begin:data_end),'LineWidth',2,'LineStyle','-');
% % % xlim([0,t]);
% % % grid on
% % % subplot(3,1,3)
% % % plot(yaw(data_begin:data_end),'LineWidth',2,'LineStyle','-');
% % % xlim([0,t]);
% % % grid on
% % % tp=length(con_in);
% % % 
% % % figure(5)
% % % plot(mindis);
% % % grid on
% % % 
% % % figure(6)
% % % plot(con_in);
% % 
