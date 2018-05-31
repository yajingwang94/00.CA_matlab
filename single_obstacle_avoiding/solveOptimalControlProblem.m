function [u, Va, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, obstCost, constraints, terminalconstraints, ...
    linearconstraints, system, N, t0, x0, u0, T, w, xd, obst, numOfObst, V)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u0, w, V);

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*T,x(k,:),u0(:,k));
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end

    % Solve optimization problem
    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    [u, Va, exitflag, output] = fmincon(@(u) costfunction(runningcosts, ...
        terminalcosts, obstCost, system, N, T, t0, x0, ...
        u, w, xd, obst, numOfObst, V), u0, A, b, Aeq, beq, lb, ...
        ub, @(u) nonlinearconstraints(constraints, terminalconstraints, ...
        system, N, T, t0, x0, u, w, obst, numOfObst, V));
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u, w, V)
    x(1,:) = x0;
    for k=1:N
        x(k+1,:) = dynamic(system, T, t0, x(k,:), u(:,k), w, V);
    end
end

function [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
             x0, u, w, V)
        x = system(t0, x0, u, T, w, V);
        x_intermediate = [x0; x];
        t_intermediate = [t0, t0+T];
end

%function x_next = system(t, x, u, T, w)
    %v = 19;
    %x_next(1) = x(1) + T/2 * (v * (cos(x(3)) + cos(x(3) + u)) + 2 * w(1));
    %x_next(2) = x(2) + T/2 * (v * (sin(x(3)) + sin(x(3) + u)) + 2 * w(2));
    %x_next(3) = x(3) + T/2 * 3 * u + 2 * w(3);
    %x_next(1) = x(1) + T/2 * (u(1) * (cos(x(3)) + cos(x(3) + u(2))) + 2 * w(1));
    %x_next(2) = x(2) + T/2 * (u(1) * (sin(x(3)) + sin(x(3) + u(2))) + 2 * w(2));
    %x_next(3) = x(3) + T/2 * 3 * u(2) + 2 * w(3);
%end

function cost = costfunction(runningcosts, terminalcosts, obstCost, system, ...
                    N, T, t0, x0, u, w, xd, obst, numOfObst, V)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, w, V);
    obst_pre = zeros(5,numOfObst); % 预测的单个时刻的所有障碍物的状态
    for k=2:N+1
         %cost = cost+runningcosts(t0+k*T, x(k,:), u(:,k));
        cost = cost + runningcosts(x(k,1:3), xd(k-1,1:3), u(:,k-1));
        for j = 1 : numOfObst
            obst_pre(1,j) = obst(1,j) + obst(4,j) * k * T * cos(obst(5,j));
            obst_pre(2,j) = obst(2,j) + obst(4,j) * k * T * sin(obst(5,j));
            obst_pre(3,j) = obst(3,j);
            obst_pre(4,j) = obst(4,j);
            obst_pre(5,j) = obst(5,j);
        end
        cost = cost + obstCost(x(k,:), obst_pre, numOfObst);
    end
    %cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:));
end

function [c,ceq] = nonlinearconstraints(constraints, ...
    terminalconstraints, system, ...
    N, T, t0, x0, u, w, obst, numOfObst, V)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, w, V);
    c = [];
    ceq = [];
    obst_pre = zeros(5,numOfObst); % 预测的单个时刻的所有障碍物的状态
    for k=1:N
        for j = 1 : numOfObst
            obst_pre(1,j) = obst(1,j) + obst(4,j) * k * T * cos(obst(5,j));
            obst_pre(2,j) = obst(2,j) + obst(4,j) * k * T * sin(obst(5,j));
            obst_pre(3,j) = obst(3,j);
            obst_pre(4,j) = obst(4,j);
            obst_pre(5,j) = obst(5,j);
        end
        [cnew, ceqnew] = constraints(t0+k*T, x(k,:), x(k+1,:), u(:,k), obst_pre, numOfObst, T, w, V);
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
   % [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:));
   % c = [c cnew];
   % ceq = [ceq ceqnew];
    
end