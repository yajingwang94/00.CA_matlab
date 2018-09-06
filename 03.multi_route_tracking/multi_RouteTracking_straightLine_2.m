function multi_RouteTracking_straightLine()
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
mpciterations = 2000;
xRef = [];
X2 = [];
Y2 = [];
XC2 = [];
YC2 = [];
PSI2 = [];
PSIF2 = [];
JVALUE2 = [];
UCON2 = [];
FLAG2 = [];
Dis2 = [];
COOPCost2 = [];
X21 = [];
Y21 = [];
%% initial states and parameters
t0 = 0;
x = 548;%200;
y = -540;%0; % in N-E-E coordination system
psi = 0;
u0 = 0 * ones(1,N);%0.3 * ones(1,N);
x0 = zeros(1,3);
step = V * T;
num_of_others = 1;
other_state = [];
other_u0 = [];
Dis = 0;
COOPCost = 0;
%% Ô²¹ì¼£·½³Ì  ÔÚX¡ª¡ªY×ø±êÏµÖÐ
     splinelength = 1400;%800;%2 * pi * R;     
     s = splinelength - 610;    % ³õÊ¼µã¶ÔÓ¦µÄ»¡³¤
    
%% communication     
    send_client = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(send_client)
    
     receive_server = tcpip('0.0.0.0', 30001, 'NetworkRole', 'server');
    fopen(receive_server);


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
        [xref(count,1), xref(count,2), xref(count,3)] = straightLine(s_temp, turns);
        count = count + 1;
    end
    %% Step (1) of the NMPC algorithm: Obtain new initial value
    x0 = [x, y, psi];     %Ã¿Ò»Ê±¿Ì³õÊ¼×´Ì¬ÔÚ±±¶«µØ×ø±êÏµÏÂµÄ×ø±ê±íÊ¾
	
     rec = fread(receive_server, 8, 'double');
     size = whos('rec');
     if(size.size>0)
     other_state(1,:) = rec(1:3);
     other_u0(1,:) = rec(4:8); 
     X21(mpciter) = rec(1);
     Y21(mpciter) = rec(2);
     end
     
    
    %% Step (2) of the NMPC algorithm: Solve the optimal control problem
    [u_new, V_current, exitflag, output, Dis, COOPCost] = solveOptimalControlProblem_multi ...
            (@runningcosts, @terminalcosts, @coopCost, @collAvoid_constraints, ...
            @terminalconstraints, @linearconstraints, @system, @coopCostTest, ...
            N, t0, x0, u0, T, wind, xref, other_state, other_u0);
    %   Store closed loop data
	u0 = shiftHorizon(u_new);
    
    X2(mpciter) = x;
    Y2(mpciter) = -y;  % Y records the coordination in X-Y system
    PSI2(mpciter) = psi;
    XC2(mpciter) = xref(1,1);
    YC2(mpciter) = - xref(1,2);
    PSIF2(mpciter) = xref(1,3);
    JVALUE2(mpciter) = V_current;
    UCON2(mpciter) = u_new(1);
    FLAG2(mpciter) = exitflag;
    Dis2(mpciter) = Dis;
    COOPCost2(mpciter) = COOPCost;
    
    %Ä¿Ç°´«ÊäµÄÐÅÏ¢Ö»°üº¬µ±Ç°Ê±¿ÌµÄ×´Ì¬ÐÅÏ¢ÒÔ¼°µ±Ç°Ê±¿ÌµÄÓÅ»¯¿ØÖÆÐòÁÐ£¬²»°üº¬Î´À´Ô¤²âÊ±ÓòÄÚµÄ²Î¿¼×´Ì¬ÐÅÏ¢ 
    send_data = zeros(1,6);
    send_data(1:3)=[x,y,psi];   %ÔÚ±±¶«µØ×ø±êÏµÏÂµÄ×ø±ê
    send_data(4:8)=u0;
    send_data 
    
    for i = 1:8
        fwrite(send_client, send_data(i), 'double') 
    end
	
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
fclose(receive_server);
fclose(send_client);

save('D:\03.code\data_analysis\multi_straightLine\UAV2_test0903_1.mat','X2','Y2', 'PSI2', 'XC2','YC2','PSIF2', 'JVALUE2', 'UCON2', 'FLAG2', 'Dis2', 'COOPCost2', 'X21', 'Y21');
%% Plot 
figure
plot(X2,Y2,'b')
hold on;
plot(XC2,YC2,'r')
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

function [x, y, psi] = straightLine(s, turns)
    % second, along the time circle
    
    if s < 100
        x = 68 + s * 3 / 5;
        y = -(176 - s * 4 / 5);
        psi = -pi + atan2(4,3);
    elseif s < 700
        x = 128 + (s - 100) * 4 / 5;
        y = -(96 + (s - 100) * 3 / 5);
        psi = pi - atan2(3,4);
    elseif s < 800
        x = 608 - (s - 700) * 3 / 5;
        y = -(456 + (s - 700) * 4 / 5);
        psi = atan2(4,3);
    else
        x = 548 - (s - 800) * 4 / 5;
        y = -(536 - (s - 800) * 3 / 5);
        psi = atan2(-3,4);
    end  
end