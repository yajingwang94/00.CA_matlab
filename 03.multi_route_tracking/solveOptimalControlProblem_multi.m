function [u, V, exitflag, output, dis, coopcost] = solveOptimalControlProblem_multi ...
    (runningcosts, terminalcosts, coopCost, collAvoid_constraints, terminalconstraints, ...
    linearconstraints, system,coopCostTest, N, t0, x0, u0, T, wind, xd, other_state, other_u0)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u0, wind);
    num_of_others = size(other_state, 1);
    if num_of_others>0
        [dis, coopcost] = coopCostTest(x0,other_state(1,:));
    else
        dis = 0;
        coopcost = 0;
    end
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
        terminalcosts, coopCost, system, N, T, t0, x0, ...
        u, wind, xd, other_state, other_u0), u0, A, b, Aeq, beq, lb, ...
        ub, @(u) nonlinearconstraints(collAvoid_constraints, terminalconstraints, ...
        system, N, T, t0, x0, u, wind, other_state, other_u0));
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u, wind)
    x(1,:) = x0;
    for k=1:N
        x(k+1,:) = dynamic(system, T, t0, x(k,:), u(:,k), wind);
    end
end

function x_others = computeOtherOpenloopSolution(system, N, T, t0, other_state, other_u0, wind)
    num_of_others = size(other_state, 1);    
    x_others = zeros(num_of_others, size(other_state, 2), N+1);
    x_others(:,:,1) = other_state;
    
    %x(1,:) = x0;
    for k=1:N
        for j = 1:num_of_others
            x_others(j,:,k+1) = system(t0, x_others(j,:,k), other_u0(j,k), T, wind);
        end
    end
end

function [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
             x0, u, wind)
        x = system(t0, x0, u, T, wind);
        x_intermediate = [x0; x];
        t_intermediate = [t0, t0+T];
end


function cost = costfunction(runningcosts, terminalcosts, coopCost, system, ...
                    N, T, t0, x0, u, wind, xd, other_state, other_u0)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, wind);
	
	%num_of_other = size(other_state,1);
    %x_other = zeros(num_of_other, size(other_state, 2), N+1);
    x_other = computeOtherOpenloopSolution(system, N, T, t0, other_state, other_u0, wind);
	
    for k=1:N
         %cost = cost+runningcosts(t0+k*T, x(k,:), u(:,k));
        cost = cost + 5*runningcosts(x(k,1:3), xd(k,1:3), u(:,k));
		
		cost = cost + coopCost(x(k+1,:), x_other(:,:,k+1));
		
    end
    cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:));
end

function [c,ceq] = nonlinearconstraints(collAvoid_constraints, ...
    terminalconstraints, system, ...
    N, T, t0, x0, u, wind, other_state, other_u0)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, T, t0, x0, u, wind);
    c = [];
    ceq = [];	
    
	%num_of_other = size(other_state,1);
    %x_other = zeros(num_of_other, size(other_state, 2), N+1);
    x_other = computeOtherOpenloopSolution(system, N, T, t0, other_state, other_u0, wind);
    %[cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:));
    %c = [c cnew];
    %ceq = [ceq ceqnew];
    for k=1:N
            [cnew, ceqnew] = collAvoid_constraints(system, x(k,:),t0, N, T, x_other(:,:,k));
            c = [c cnew];
            ceq = [ceq ceqnew];
     end
	
	
	
end