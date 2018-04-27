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

% scatter(u_in,err_t,'k')
figure(1)
plot(u_in,err_t,'sk','markerface','k','markersize',10);
xlabel({' \Sigma|U|^2'},'FontSize',20,'FontName','Times New Roman');
ylabel({' \Sigma|E|^2'},'FontSize',20,'FontName','Times New Roman');
for i=1:9
    text(u_in(i)+100,err_t(i)-0,num2str(lab(i)),'fontsize',20,'color','b')
end

figure(2)
plot(fu_in,ferr_t,'sk','markerface','k','markersize',10);
xlabel({' \Sigma|U|'},'FontSize',20,'FontName','Times New Roman');
ylabel({' \Sigma|E|'},'FontSize',20,'FontName','Times New Roman');
for i=1:9
    text(fu_in(i)+20,ferr_t(i)-0,num2str(lab(i)),'fontsize',20,'color','b')
end



