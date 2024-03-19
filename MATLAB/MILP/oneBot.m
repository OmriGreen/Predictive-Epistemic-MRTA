% clear, pack, clc
% References:
% https://aleksandarhaber.com/solve-mixed-integer-linear-programming-milp-optimization-problems-in-matlab/,
% https://www.mathworks.com/help/optim/ug/travelling-salesman-problem.html
%Techniques for Subtour Elimination in Traveling Salesman Problem: Theory and Implementation in Python   
%Multi-robot long-term persistent coverage with fuel constrained robots
    %I1, I2, A1, A2, A3

clear, clc

% Scenario Setup
S = 50; % The max X and max Y positions
startLoc = [0 0]; % Start Location [x y]
endLoc = [0 0]; % End Location [x y]
T = 5; % Number of Tasks

% Cost Vector
cV = costVector(T, S);
f = transpose(cV);

% Linear Equality Constraint Equation
Aeq = calcAeq(T);
beq = calcBeq(T);

% Integer Variables
intcon = 1:length(cV);

% Lower bound constraint (NO NEGATIVE NUMBERS)
lb = zeros(length(cV),1);

% Upper Bound value
ub = ones(length(cV), 1) * inf;
ub((T-1)^2+1) = T-1; % Adjust upper bound for the last variable

% Inequality Constraint Equation
A = calcIeq(T);
b = calcBIeq(T);

% Adjust dimensions of f to match Aeq
f = [f; zeros(size(lb, 1) - length(f), 1)];

%Solve MILP with updated options
rawX = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub);
x = [];
tX = [];
for i=1:length(rawX)
    tX = [tX rawX(i)];
    if rem(length(tX),T)==0
        x = [x; tX];
        tX = [];
    end
end
x

A5 = req5(T)


%% ======= Linear Inequality Constraints ======
%Calculate Equality Constraints
function bI=calcBIeq(T)
    bI = [];    
    %I1: sum_j(x_Sj)<=1
    bI = [bI;1];
    %I2: sum_i(x_iS)<=1
    bI=[bI;1];

end

%Calculate the Inequality Matrix
function I=calcIeq(T)
   %% Degree Constraints
   %I1: sum_j(x_Sj)<=1
    I1 = getI1(T);

    %I2: sum_i(x_iS)<=1
    I2 = getI2(T);

   I=[I1;I2];
end

%I1: sum_j(x_Sj)<=1
function I1=getI1(T)
    I1 = [];
    for j = 1:T
        for i = 1:T
            if i==1
                I1 = [I1 1];
            else
                I1 = [I1 0];
            end
        end
    end
    I1=[I1 zeros(1,(T)^2)];
end

%I1: sum_i(x_iS)<=1
function I2=getI2(T)
    I2 = [];
    for j = 1:T
        for i = 1:T
            if j==1
                I2 = [I2 1];
            else
                I2 = [I2 0];
            end
        end
    end
    I2 = [I2 zeros(1,(T)^2)];
end





%% ------------Linear Equality Constraints-------------
%Creates all necessary equality constraints
function beq = calcBeq(T)
    beq = [];
    %%============== z constraints====================================
    % A1: for all j sum_i(z_ij)=1
    for i=1:T
        beq = [beq;1];
    end

    % A2: for all i sum_jS(z_ij)=1
    for i=1:T
        beq = [beq;1];
    end

    %A3: for all j sum_i
    for i=1:T
        beq = [beq;0];
    end

    %A4: Flow Through Starting Node
    beq = [beq; 0];

    %A5: Energy Updated after visting each node
    for i=1:T
        beq = [beq;0];
    end

end

% Creates the Equality Matrix
function Aeq = calcAeq(T)
    % Calculates all rows of A for the given number of Tasks
    %% Degree Constraints
    A1 = req1(T); % for all i sum_j(z_ij)=1
    A2 = req2(T); % for all j sum_i(z_ij)=1
    A3 = req3(T); % for all j sum_i(z_ij-z_ji)=0

    %% Flow Constraints
    %A4: Flow Through Starting Node
    A4 = req4(T);

    %A5: Energy Updated after visiting each node
    A5 = req5(T);
    Aeq = [A1;A2;A3;A4;A5];
end
    
%A5: Energy Updated after visiting each node
function A5 = req5(T)
    A5 = [];
    for iC=1:T
        tA = [];
        %Goes over z
        for j = 1:T
            for i = 1:T
                if i == iC
                    tA = [tA -1];
                else
                    tA = [tA 0];
                end
                
            end
        end

        %Goes Through y
        for j = 1:T
            for i = 1:T
                if j == iC && i ~= j
                    tA = [tA 1];
                else
                    if i == iC && i ~= j
                        tA = [tA -1];
                    else
                        tA = [tA 0];
                    end
                end
            end
        end
        A5 =[A5; tA];
    end
end

%A4: Flow Through Starting Node
function A4 = req4(T)
    %Gets sum_i,h(z_ij)
    A4 = -ones(1,(T)^2);
    
    %Gets sum_i(y_Si-y_iS)
    for j = 1:T
        for i = 1:T
            if i==1 && i~=j
                A4 = [A4 1];
            else
                if j == 1 && i~=j
                    A4 = [A4 -1];
                else
                    A4 = [A4 0];
                end
            end
        end
    end
end


% for all j sum_i(x_ij-x_ji)=0
function A3 = req3(T)
    A3 = [];
    for jC=1:T
        tA = [];
        for j = 1:T
            for i = 1:T
                if j == jC
                    tA = [tA 1];
                else
                    if i == jC
                        tA = [tA -1];
                    else
                        tA = [tA 0];
                    end
                end
            end
        end
        tA = [tA zeros(1,(T)^2)];
        A3 = [A3; tA];
    end
end

% for all i sum_j(z_ij)=1
function A2 = req2(T)
    A2 = [];

    for i=1:T
            tA = [];
            for j=1:T
                for k=1:T
                    if k==i
                        tA = [tA 1];
                    end
                    if k~=i
                        tA = [tA 0];
                    end
                end
            end
            tA = [tA zeros(1,(T)^2)]; 
            A2 = [A2;tA];
    end
end

% for all j sum_i!=E(z_ij)=1
function A1 = req1(T)
    A1 = [];

    for i=1:T
        tA = [];
        for j=1:T
            for k=1:T
                if j==i
                    tA = [tA 1];
                end
                if j~=i
                    tA = [tA 0];
                end
            end
        end
        tA = [tA zeros(1,(T)^2)];
        A1 = [A1;tA];
    end
end

%% ---------Cost Vector Functions----------

%Creates a cost Vector for a MILP problem
function cV = costVector(T, S)
    cV = [];
    locations = setup(T, S);
    
    for i=1:T
        for j=1:T
            x1=locations(i,1);
            x2=locations(j,1);
            y1=locations(i,2);
            y2=locations(j,2);
            cV=[cV findDist(x1,x2,y1,y2)];
        end
    end
    cV = [cV zeros(1,(T)^2)];
end

%Finds the distance between two points
function distance = findDist(x1,x2,y1,y2)
    distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end

%Creates Locations for all tasks with a set amount of tasks
function taskLocations = setup(T, S)
    maxSize = S;
    taskLocations = [];
    for i=1:T
        a = floor(maxSize*rand);
        b = floor(maxSize*rand);
        taskLocations = [taskLocations; [a,b]];
    end
end