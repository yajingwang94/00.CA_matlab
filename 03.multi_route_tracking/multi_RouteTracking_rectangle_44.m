function multi_RouteTracking_rectangle()
clear all
clc
%% global veriables
N = 5;
T = 0.05;
xref = zeros(N,4);
wind = [0, 0, 0];
%wind = [2.12132034,2.12132034,0];
%wind = [3.53533906, 3.535533906,0];
%wind = [5.65685425, 5.65685425,0];
%wind = [7.07106781, 7.07106781,0];
%wind = [1, 1, 0];
V = 19;
mpciterations = 1600;
xRef = [];
X4 = [];
Y4 = [];
XC4 = [];
YC4 = [];
PSI4 = [];
PSIF4 = [];
JVALUE4 = [];
UCON4 = [];
FLAG4 = [];
Dis4 = [];
COOPCost4 = [];
X4rec = [];
Y4rec = [];
%% initial states and parameters
t0 = 0;
x = 530;%200;
y = -570;%0; % in N-E-E coordination system
psi = 0;
u0 = -0.3 * ones(1,N);%0.3 * ones(1,N);
x0 = zeros(1,3);
step = V * T;
num_of_others = 1;
other_state = [];
other_u0 = [];
Dis = 0;
COOPCost = 0;
%% Ô²¹ì¼£·½³Ì  ÔÚX¡ª¡ªY×ø±êÏµÖÐ
     splinelength = 1880;%800;%2 * pi * R;     
     s = splinelength - 400;    % ³õÊ¼µã¶ÔÓ¦µÄ»¡³¤
    
%% communication   
    remotePort = 30003;  
    udpSender = udp('192.168.43.255',remotePort);
    set(udpSender,'OutputBufferSize',1048576);
    set(udpSender,'TimeOut',100);
    fopen(udpSender);

    portUAV1 = 30000;
    portUAV2 = 30001;
    portUAV3 = 30002;

    udpReceiver1 = udp('','LocalPort',portUAV1);
    %set(udpReceiver1,'OutputBufferSize',8192);
    set(udpReceiver1,'TimeOut',100);
    udpReceiver1.EnablePortSharing = 'on';
    fopen(udpReceiver1);

    udpReceiver2 = udp('','LocalPort',portUAV2);
    %set(udpReceiver2,'OutputBufferSize',8192);
    set(udpReceiver2,'TimeOut',100);
    udpReceiver2.EnablePortSharing = 'on';
    fopen(udpReceiver2);

    udpReceiver3 = udp('','LocalPort',portUAV3);
    %set(udpReceiver3,'OutputBufferSize',8192);
    set(udpReceiver3,'TimeOut',100);
    udpReceiver3.EnablePortSharing = 'on';
    fopen(udpReceiver3);
    
     


%% The iterative process
mpciter = 1;
while(mpciter < mpciterations)

    turns = 1;
    if s <= 10
       s = splinelength - 10;   % Ë³Ê±Õë·½Ïò£¬s¼õÐ¡
		%break;
        turns = turns+1;
    end
    %%%%% The prediction of the N reference points 
    count = 1;
    s_temp = 0;
    while count <= N
        s_temp = s - (count - 1) * step;
        [xref(count,1), xref(count,2), xref(count,3)] = rectangle(s_temp, turns);
        count = count + 1;
    end
    %% Step (1) of the NMPC algorithm: Obtain new initial value
    x0 = [x, y, psi];     %Ã¿Ò»Ê±¿Ì³õÊ¼×´Ì¬ÔÚ±±¶«µØ×ø±êÏµÏÂµÄ×ø±ê±íÊ¾
	
%communication
    send_data = zeros(1,8);
    %(1) = 1;
    send_data(1:3)=x0;
    send_data(4:8)=u0;
    send_data
    fwrite(udpSender,send_data,'float');

     rec_1 = fread(udpReceiver1, 8, 'float')
     size = whos('rec_1');
     if(size.size>0)
     other_state(1,:) = rec_1(1:3);
     other_u0(1,:) = rec_1(4:8);
     X4rec(1,mpciter)=rec_1(1);
     Y4rec(1,mpciter)=rec_1(2);
     end
     
     rec_2 = fread(udpReceiver2, 8, 'float')
     size = whos('rec_2');
     if(size.size>0)
     other_state(2,:) = rec_2(1:3);
     other_u0(2,:) = rec_2(4:8);
     X4rec(2,mpciter)=rec_2(1);
     Y4rec(2,mpciter)=rec_2(2);
     end

     rec_3 = fread(udpReceiver3, 8, 'float')
     size = whos('rec_3');
     if(size.size>0)
     other_state(3,:) = rec_3(1:3);
     other_u0(3,:) = rec_3(4:8);
     X4rec(3,mpciter)=rec_3(1);
     Y4rec(3,mpciter)=rec_3(2);
     end
     
    
    %% Step (2) of the NMPC algorithm: Solve the optimal control problem
    [u_new, V_current, exitflag, output, Dis, COOPCost] = solveOptimalControlProblem_multi ...
            (@runningcosts, @terminalcosts, @coopCost, @collAvoid_constraints, ...
            @terminalconstraints, @linearconstraints, @system, @coopCostTest, ...
            N, t0, x0, u0, T, wind, xref, other_state, other_u0);
    %   Store closed loop data
	u0 = shiftHorizon(u_new);
    
    X4(mpciter) = x;
    Y4(mpciter) = -y;  % Y records the coordination in X-Y system
    PSI4(mpciter) = psi;
    XC4(mpciter) = xref(1,1);
    YC4(mpciter) = - xref(1,2);
    PSIF4(mpciter) = xref(1,3);
    JVALUE4(mpciter) = V_current;
    UCON4(mpciter) = u_new(1);
    FLAG4(mpciter) = exitflag;
    Dis4(mpciter) = Dis;
    COOPCost4(mpciter) = COOPCost;
    
    t0 = t0 + T;
    %% Step (3) of the NMPC algorithm: Apply control to process
    x = x0(1) + T/2 * (V * (cos(x0(3)) + cos(x0(3) + u_new(1))) + 2 * wind(1));
    y = x0(2) + T/2 * (V * (sin(x0(3)) + sin(x0(3) + u_new(1))) + 2 * wind(2));
    psi = x0(3) + T/2 * 3 * u_new(1) + 2 * wind(3);
	    
    if psi > pi
            psi = psi-2*pi;
    elseif psi < -pi
            psi = psi+2*pi;
    end	
	
    ds = sqrt((x - xref(1,1))^2 + (y - xref(1,2))^2);
    if ds > 30
            s = s;
    else    % Ë³Ê±Õë
            s = s - 15;
    end  
    mpciter = mpciter+1 
end
fclose(udpSender)
delete(udpSender)
fclose(udpReceiver1)
delete(udpReceiver1)
fclose(udpReceiver2)
delete(udpReceiver2)
fclose(udpReceiver3)
delete(udpReceiver3)

save('D:\05.Code\data_analysis\four_UAV_test\rectangle0912_1_uav4.mat','X4','Y4', 'PSI4', 'XC4','YC4','PSIF4', 'JVALUE4', 'UCON4', 'FLAG4', 'Dis4', 'COOPCost4', 'X4rec', 'Y4rec');
%% Plot 
figure
plot(X4,Y4,'b')
hold on;
plot(XC4,YC4,'r')
hold on;
axis equal;

end

% need thinking % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function cost = runningcosts(x, xd, u)
    %cost = norm(x - xd, 2)^2;%+norm(u,2)^2;
    %cost = (x(1) - xd(1))^2 + (x(2) - xd(2))^2;
    cost = norm(x - xd, 2)^2;
end

function cost = terminalcosts(t, x)

    cost = 0.0;
end

function cost = coopCost(x, x_others)
    k1 = 100;
    %k2 = 25;
    sigma = 100;%200;
    cost = 0.0;
	numOfOthers = size(x_others,1);
    for j = 1: numOfOthers
        d_2 = (x_others(j,1) -  x(1))^2 + (x_others(j,2) -  x(2))^2;
        if d_2 > 40000
            d_2 = 0;
            J_coop = 0;
        else
            J_coop = k1 * exp(-d_2/(sigma^2));
        end
        cost = cost + J_coop;
    end
end

function [dis, COOPCost] = coopCostTest(x0, other_state)
    k1 = 100;
    %k2 = 25;
    sigma = 100;%200;
	%numOfOthers = size(x_others,1);
    COOPCost = zeros(1,3); %µ±Ç°×´Ì¬µÄ´ú¼Û
    %dis = zeros(1,3);
    %for j = 1: numOfOthers
        d_2 = (other_state(1,1) -  x0(1))^2 + (other_state(1,2) -  x0(2))^2;
        dis = sqrt(d_2);
        if d_2 > 40000
            d_2 = 0;
            J_coop = 0;
        else
            J_coop = k1 * exp(-d_2/(sigma^2));
        end;
        COOPCost = J_coop;
       % cost = cost + J_coop;
    %end
end


function [c,ceq] = collAvoid_constraints(system, x,t0, N, T, x_other)
    c = 0;
	d_safe = 40;
    num_of_others = size(x_other, 1);
	for i = 1:num_of_others
		dis_2 = (x(1) - x_other(i,1))^2 + (x(2) - x_other(i,2))^2;
		
		c_add = d_safe^2 - dis_2;
        c   = c + c_add;
	end
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x)
    c   = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    %lb  = [15;-3];
    %ub  = [23;3];
    lb  = [-0.3];
    ub  = [0.3];
end


function x_next = system(t, x, u, T, wind)
    v = 19;
    x_next(1) = x(1) + T/2 * (v * (cos(x(3)) + cos(x(3) + u)) + 2 * wind(1));
    x_next(2) = x(2) + T/2 * (v * (sin(x(3)) + sin(x(3) + u)) + 2 * wind(2));
    x_next(3) = x(3) + T/2 * 3 * u + 2 * wind(3);
end

function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end



function [xref, yref, psiref] = getRef_2(s, N, R)
% Ë³Ê±Õë·½Ïò¼ÆËã²Î¿¼µãºÍ²Î¿¼º½Ïò½Çalong the direction of the circle
     theta = s / R;
      if theta > pi
                theta = theta - 2 * pi;
        elseif theta < - pi
                theta = theta + 2 * pi;
        end	
     xref = R * cos(theta);
     yref = - R * sin(theta); % µÃµ½²Î¿¼µãÔÚ±±¶«µØ×ø±êÏµÏÂµÄ×ø±ê
     %xprim = -sin(theta);
     %yprim = cos(theta);
    % psiref = atan2(yprim,xprim);
     %psiref = atan2(yprim,xprim) + 2*pi;
     if theta >= 0 
         psiref = pi/2 - theta;
     elseif theta >= -pi/2 
         psiref = pi/2 - theta;
     else
         psiref = -3*pi/2 - theta;
     end    %µÃµ½ÏÂÒ»¸ö²Î¿¼µãµÄ²Î¿¼º½Ïò½Ç£¨±±¶«µØ×ø±êÏµÏÂ£©
     %if psiref > pi
     %           psiref = psiref - 2 * pi;
     %elseif psiref < - pi
     %           psiref = psiref + 2 * pi;
    % end
     
end

function [x, y, psi] = rectangle(s, turns)
    % second, along the time circle
    if s < 220
        x = -44 + s * 3 / 5;
        y = -(192 - s * 4 / 5);
        psi = -pi + atan2(4,3);
    elseif s < 940
        x = 88 + (s - 220) * 4 / 5;
        y = -(16 + (s - 220) * 3 / 5);
        psi = pi - atan2(3,4);
    elseif s < 1160
        x = 664 - (s - 940) * 3 / 5;
        y = -(448 + (s - 940) * 4 / 5);
        psi = atan2(4,3);
    else
        x = 532 - (s - 1160) * 4 / 5;
        y = -(624 - (s - 1160) * 3 / 5);
        psi = atan2(-3,4);
    end
end