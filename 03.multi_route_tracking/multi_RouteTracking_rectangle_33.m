function multi_RouteTracking_rectangle()
clear all
clc
%% global variebles
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
mpciterations =1600;
xRef = [];
X3 = [];
Y3 = [];
XC3 = [];
YC3 = [];
PSI3 = [];
PSIF3 = [];
JVALUE3 = [];
UCON3= [];
FLAG3 = [];
Dis3 = [];
COOPCost3 = [];
X3rec = [];
Y3rec = [];
%% initial states and parameters
t0 = 0;
x = 90;%30;
y = -40;%0;     %³õÊ¼Ê±¿ÌµÄÎ»ÖÃÔÚ±±¶«µØ×ø±êÏµÏÂµÄ±íÊ¾
psi = -1.57;
u0 = 0.3 * ones(1,N);
x0 = zeros(1,3);
step = V * T;
num_of_others = 1;
other_state = [];
other_u0 = [];
Dis = 0;
COOPCost = 0;

%% Ô²¹ì¼£·½³Ì
%     R = 300;
     splinelength = 1720;%2 * pi * R;     
     s = 210;    
     
%% communication   
    remotePort = 30002;  
    udpSender = udp('192.168.43.255',remotePort);
    set(udpSender,'OutputBufferSize',2097152);
    set(udpSender,'TimeOut',100);
    fopen(udpSender);

    portUAV1 = 30000;
    portUAV2 = 30001;
    portUAV4 = 30003;

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

    udpReceiver3 = udp('','LocalPort',portUAV4);
    %set(udpReceiver3,'OutputBufferSize',8192);
    set(udpReceiver3,'TimeOut',100);
    udpReceiver3.EnablePortSharing = 'on';
    fopen(udpReceiver3);

%% The iterative process
mpciter = 1;
while(mpciter < mpciterations)

    turns = 1;
    if s >= splinelength
        s = 10;  %ÄæÊ±Õë·½Ïò£¬sÔö´ó 
		%break;
        turns = turns+1;
    end
	
    %%%%% The prediction of the N reference points 
    count = 1;
    s_temp = 0;
    while count <= N
        s_temp = s + (count - 1) * step;
        [xref(count,1), xref(count,2), xref(count,3)] = rectangle(s_temp, turns);		
        count = count + 1;
    end
    %% Step (1) of the NMPC algorithm: Obtain new initial value
    x0 = [x, y, psi];    %Ã¿Ò»Ê±¿Ì³õÊ¼×´Ì¬ÔÚ±±¶«µØ×ø±êÏµÏÂµÄ×ø±ê±íÊ¾

%communication
    send_data = zeros(1,8);
    %(1) = 1;
    send_data(1:3)=x0;
    send_data(4:8)=u0;
    send_data
    fwrite(udpSender,send_data,'float')


     rec_1 = fread(udpReceiver1, 8, 'float')
     size = whos('rec_1');
     if(size.size>0)
     other_state(1,:) = rec_1(1:3);
     other_u0(1,:) = rec_1(4:8);
     X3rec(1,mpciter)=rec_1(1);
     Y3rec(1,mpciter)=rec_1(2);
     end
     
     rec_2 = fread(udpReceiver2, 8, 'float')
     size = whos('rec_2');
     if(size.size>0)
     other_state(2,:) = rec_2(1:3);
     other_u0(2,:) = rec_2(4:8);
     X3rec(2,mpciter)=rec_2(1);
     Y3rec(2,mpciter)=rec_2(2);
     end

     rec_3 = fread(udpReceiver3, 8, 'float')
     size = whos('rec_3');
     if(size.size>0)
     other_state(3,:) = rec_3(1:3);
     other_u0(3,:) = rec_3(4:8);
     X3rec(3,mpciter)=rec_3(1);
     Y3rec(3,mpciter)=rec_3(2);
     end
       
    
    %% Step (2) of the NMPC algorithm: Solve the optimal control problem
    [u_new, V_current, exitflag, output, Dis, COOPCost] = solveOptimalControlProblem_multi ...
            (@runningcosts, @terminalcosts, @coopCost, @collAvoid_constraints, ...
            @terminalconstraints, @linearconstraints, @system, @coopCostTest,...
            N, t0, x0, u0, T, wind, xref, other_state, other_u0);
    %   Store closed loop data
	u0 = shiftHorizon(u_new);
    
    X3(mpciter) = x;
    Y3(mpciter) = -y; % Y records the coordination in X-Y coordination system
    PSI3(mpciter) = psi;
    XC3(mpciter) = xref(1,1);
    YC3(mpciter) = -xref(1,2);
    PSIF3(mpciter) = xref(1,3);
    JVALUE3(mpciter) = V_current;
    UCON3(mpciter) = u_new(1);
    FLAG3(mpciter) = exitflag;
    Dis3(mpciter) = Dis;
    COOPCost3(mpciter) = COOPCost;
    
    
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
    else
            s = s + 15;
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

save('D:\05.Code\data_analysis\four_UAV_test\rectangle0912_1_uav3.mat','X3','Y3', 'PSI3', 'XC3','YC3','PSIF3', 'JVALUE3','UCON3', 'FLAG3', 'Dis3', 'COOPCost3', 'X3rec', 'Y3rec' );
%% Plot 
figure
%plot(xctlp,yctlp,':'); % »æÖÆ¿ØÖÆ¶à±ßÐÎ£»
%hold on; % Ä¬ÈÏÎªhold off£¬´ËÃüÁîÓÃÀ´±£Áô¿ØÖÆ¶à±ßÐÎµÄÍ¼ÐÎ£»

plot(X3,Y3,'b')
hold on;
%plot(X1,Y1,'g')
%hold on;
plot(XC3,YC3,'r')
hold on;
axis equal;

end

% need thinking % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function cost = runningcosts(x, xd, u)
    %cost = (x(1) - xd(1))^2 + (x(2) - xd(2))^2;%+norm(u,2)^2;
    cost = norm(x - xd, 2)^2;
end

function cost = terminalcosts(t, x)

    cost = 0.0;   % how to calculate and judge
end

function cost = coopCost(x, x_others)
    k1 = 200;
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
	d_safe = 50;
    num_of_others = size(x_other, 1);
	for i = 1:num_of_others
		dis_2 = (x(1) - x_other(i,1))^2 + (x(2) - x_other(i,2))^2;
		if dis_2 > 10000
            c_add = 0;
        else
            c_add = d_safe^2 - dis_2;
        end
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

function [xref, yref, psiref] = getRef(s, N, R)
% ÄæÊ±Õë·½Ïò¼ÆËã²Î¿¼µãµÄ×ø±êºÍ²Î¿¼º½Ïò½Çreverse direction of the circle
     theta = s / R;
      if theta > pi
                theta = theta - 2 * pi;
        elseif theta < - pi
                theta = theta + 2 * pi;
        end	
     xref = R * cos(theta);
     yref = - R * sin(theta); % use the N-E-E coordination
     %xprim = -sin(theta);
     %yprim = cos(theta);
     %psiref = atan2(yprim,xprim);
     if theta <= 0 
        psiref = - pi/2 - theta ;
    elseif theta <= pi/2
        psiref = - pi/2 - theta;
     else
            psiref = 3*pi/2 - theta;
        end
end

function [x, y, psi] = straightLine(s, turns)
    % input is the arc length
    % climb first, drop later
    k = 3;  % choose all k to be positive
    b = 5;
    x = s / sqrt(1+k^2);
    y = -(k * x + b); % output the coordination in NED system
    sig = mod(turns, 2);
    if k > 0
    	if sig == 0   % drop
    		psi = pi - atan(k);
    	else   %climb
    		psi = -atan(k);
    	end
    elseif k < 0
    	if sig == 0  % drop
    		psi = - atan(k);
    	else       % sig = 1  climb
    		psi = -pi - atan(k);
    	end
    end
end


function [x, y, psi] = rectangle(s, turns)
% the first, reverse the time circle
    if s < 180
    	x = -16 + s * 3 / 5;
    	y = -(188 - s * 4 / 5);
    	psi = atan2(4,3);
    elseif s < 860
    	x = 92 + (s - 180) * 4 / 5;
    	y = -(44 + (s - 180) * 3 / 5);
    	psi = atan2(-3,4);
    elseif s < 1040
    	x = 636 - (s - 860) * 3 / 5;
    	y = -(452 + (s - 860) * 4 / 5);
    	psi = - pi + atan2(4,3);
    else
    	x = 528 - (s - 1040) * 4 / 5;
    	y = -(596 - (s - 1040) * 3 / 5);
    	psi = pi - atan2(3,4);
    end
end