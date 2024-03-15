% clear, pack, clc
% References:
% https://aleksandarhaber.com/solve-mixed-integer-linear-programming-milp-optimization-problems-in-matlab/,
% https://www.mathworks.com/help/optim/ug/travelling-salesman-problem.html
%Techniques for Subtour Elimination in Traveling Salesman Problem: Theory and Implementation in Python   


clear, clc

% Scenario Setup
S = 50; % The max X and max Y positions
startLoc = [0 0]; % Start Location [x y]
endLoc = [0 0]; % End Location [x y]
T = 3; % Number of Tasks
B = 99999; %Large number for subtour elimination

% Cost Vector
cV = costVector(T, startLoc, endLoc, S);
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
ub((T+1)^2+1) = T+1; % Adjust upper bound for the last variable

% Inequality Constraint Equation
A = calcIeq(T,B);
b = calcBIeq(T,B);

% Adjust dimensions of f to match Aeq
f = [f; zeros(size(lb, 1) - length(f), 1)];

%Solve MILP with updated options
rawX = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub);
x = [];
tX = [];
for i=1:length(rawX)
    tX = [tX rawX(i)];
    if rem(length(tX),T+2)==0
        x = [x; tX];
        tX = [];
    end
end
x

    cc = getCC(T,B);

%% ======= Linear Inequality Constraints ======
%Calculate Equality Constraints
function bI=calcBIeq(T,B)
    bI = [];
    %% Continuity Constraint
    %for all ij y_i-B(1-x_ij)-y_j<=0
    for i = 1:(T+1)^2-(T+1)
        %bI=[bI;-B];
    end
        
end

%Calculate the Inequality Matrix
function I=calcIeq(T,B)
    %% Continuity Constraint
    %y_j>=y_i-B(1-z_ij) i,j=[2,...T+2]->B>=y_i-y_j+Bz_ij i,j=[2,...T+2]
    cc = getCC(T,B);

    I=[cc];
end

%y_j>=y_i-B(1-z_ij) i,j=[2,...T+2]->B>=y_i-y_j+Bz_ij i,j=[2,...T+2]
function CC = getCC(T,B)
    CC = [];

end




%% ------------Linear Equality Constraints-------------
%Creates all necessary equality constraints
function beq = calcBeq(T)
    beq = [];
    %%============== z constraints====================================
    % A1: for all j sum_i!=E(z_ij)=1
    for i=1:T+1
        beq = [beq;1];
    end

    % A2: for all i sum_j!=S(z_ij)=1
    for i=1:T+1
        beq = [beq;1];
    end

    %A3: for all i z_ii=0->sum_i(z_ii)=0
    beq = [beq; 0];

    %A4: y_S = T+1
    beq = [beq; T+1];

    %A5: y_E = 0
    beq = [beq; 0];
  
end

% Creates the Equality Matrix
function Aeq = calcAeq(T)
    % Calculates all rows of A for the given number of Tasks
    %% z Constraints
    A1 = req1(T); % for all i sum_j(z_ij)=1
    A2 = req2(T); % for all j sum_i(z_ij)=1
    A3 = req3(T); %for all i z_ii=0 -> sum_i(z_ii)=0
    A4 = req4(T); %y_S=T+1
    A5 = req5(T); %y_E=0

    Aeq = [A1;A2;A3; A4; A5];
end

%y_S=T+1
function A5 = req5(T)
    A5 = [];
    for j = 1:T+2
        for i = 1:T+2
            A5 = [A5 0];
        end
    end
    for y = 1:T+2
        if y == T+2
            A5 = [A5 1];
        else
            A5 = [A5 0];
        end
    end
end

%y_S=T+1
function A4 = req4(T)
    A4 = [];
    for j = 1:T+2
        for i = 1:T+2
            A4 = [A4 0];
        end
    end
    for y = 1:T+2
        if y == 1
            A4 = [A4 1];
        else
            A4 = [A4 0];
        end
    end
end

%for all i z_ii=0 -> sum_i(z_ii)=0
function A3 = req3(T)
    A3 = [];
    for j = 1:T+2
        for i = 1:T+2
            if j==i
                A3 = [A3 1];
            else
                A3 = [A3 0];
            end
        end
    end
    for y = 1:T+2
        A3 = [A3 0];
    end
end

% for all i sum_j!=S(z_ij)=1
function A2 = req2(T)
    A2 = [];

    for i=1:T+2
        if i~=1
            tA = [];
            for j=1:T+2
                for k=1:T+2
                    if k==i
                        tA = [tA 1];
                    end
                    if k~=i
                        tA = [tA 0];
                    end
                end
            end
            for z=1:T+2
                tA = [tA 0];
            end 
            A2 = [A2;tA];
        end
    end
end

% for all j sum_i!=E(z_ij)=1
function A1 = req1(T)
    A1 = [];

    for i=1:T+1
        tA = [];
        for j=1:T+2
            for k=1:T+2
                if j==i
                    tA = [tA 1];
                end
                if j~=i
                    tA = [tA 0];
                end
            end
        end
        for z=1:T+2
            tA = [tA 0];
        end 
        A1 = [A1;tA];
    end
end

%% ---------Cost Vector Functions----------

%Creates a cost Vector for a MILP problem
function cV = costVector(T,startLoc,endLoc, S)
    cV = [];
    locations = setup(T, startLoc, endLoc, S);
    
    for i=1:T+2
        for j=1:T+2
            x1=locations(i,1);
            x2=locations(j,1);
            y1=locations(i,2);
            y2=locations(j,2);
            cV=[cV findDist(x1,x2,y1,y2)];
        end
    end
    for i=1:T+2
        cV = [cV 0];
    end
end

%Finds the distance between two points
function distance = findDist(x1,x2,y1,y2)
    distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end

%Creates Locations for all tasks with a set amount of tasks
function taskLocations = setup(T, startLoc, endLoc, S)
    maxSize = S;
    taskLocations = [];
    taskLocations = [taskLocations; startLoc];
    for i=1:T
        a = floor(maxSize*rand);
        b = floor(maxSize*rand);
        taskLocations = [taskLocations; [a,b]];
    end
    taskLocations = [taskLocations;endLoc];
end
