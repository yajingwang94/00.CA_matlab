function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system, N, t0, x0, u0, T, w, xd)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u0, w);

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
    [u, V, exitflag, output] = fmincon(@(u) costfunction(runningcosts, ...
        terminalcosts, system, N, T, t0, x0, ...
        u, w, xd), u0, A, b, Aeq, beq, lb, ...
        ub, @(u) nonlinearconstraints(constraints, terminalconstraints, ...
        system, N, T, t0, x0, u, w));
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u, w)
    x(1,:) = x0;
    for k=1:N
        x(k+1,:) = dynamic(system, T, t0, x(k,:), u(:,k), w);
    end
end

function [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
             x0, u, w)
        x = system(t0, x0, u, T, w);
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

function cost = costfunction(runningcosts, terminalcosts, system, ...
                    N, T, t0, x0, u, w, xd)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, w);
    for k=1:N
         %cost = cost+runningcosts(t0+k*T, x(k,:), u(:,k));
        cost = cost + runningcosts(x(k,1:3), xd(k,1:3), u(:,k));
    end
    cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:));
end

function [c,ceq] = nonlinearconstraints(constraints, ...
    terminalconstraints, system, ...
    N, T, t0, x0, u, w)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, w);
    c = [];
    ceq = [];
    for k=1:N
        [cnew, ceqnew] = constraints(t0+k*T,x(k,:),u(:,k));
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:));
    c = [c cnew];
    ceq = [ceq ceqnew];
end