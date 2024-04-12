% clear, pack, clc
% References:
% https://aleksandarhaber.com/solve-mixed-integer-linear-programming-milp-optimization-problems-in-matlab/,
% https://www.mathworks.com/help/optim/ug/travelling-salesman-problem.html
%Techniques for Subtour Elimination in Traveling Salesman Problem: Theory and Implementation in Python   
%Multi-robot long-term persistent coverage with fuel constrained robots
%[OR1-Modeling] Lecture 3: Integer Programming #11 Traveling salesperson problem: Subtour elimination
    %https://www.youtube.com/watch?v=-m7ASCB0a8E
    %Time=5:10

clear, clc

% Scenario Setup
S = 50; % The max X and max Y positions
T = 7; % Number of Tasks

% Cost Vector
cV = costVector(T, S);
f = transpose(cV);

% Linear Equality Constraint Equation
Aeq = calcAeq(T);
beq = calcBeq(T);

% Integer Variables
intcon = 1:length(cV);

% Lower bound constraint (NO NEGATIVE NUMBERS)
lb = [zeros(1,T^2),1,2*ones(1,T-1)]; %for all i,j min(z_ij) = 0, y_S=1, y_i >=2 i = {2,...,T} 

% Upper Bound value
ub = [ones(1,T^2),1,T*ones(1,T-1)]; %for all i,j max(z_ij)=1, y_S = 1, y_i <=T i = {2,...,T}

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

z1 = getZ1(T);
z2 = getZ2(T);
z3 = getZ3(T);
z4 = getZ4(T);


%% ======= Linear Inequality Constraints ======
%Calculate Equality Constraints
function bI=calcBIeq(T)
    bI = [];    
    for i = 1:(T-1)^2
        bI = [bI; T-2];
    end

end

%Calculate the Inequality Matrix
function I=calcIeq(T)
   %% Flow Constraint
   I=getFC(T);
end

%y_j-y_i +(T-1)z_ij <= T-2 i,j={2,...T}
function fC = getFC(T)
    fC = [];

    %i,j={2,...T}
    for jC = 2:T
        for iC = 2:T
            tA = [];
            %Goes Through z
            for j = 1:T
                for i = 1:T
                    %(T-1)z_ij
                    if i==iC && j==jC
                        tA = [tA T-1];
                    else
                        tA = [tA 0];
                    end
                end
            end
            %Creates y
            for n = 1:T
                %If iC=jC y_i-y_n=0 therefore y_iC=0
                %-y_i
                if n == iC && iC~=jC
                    tA = [tA -1];
                else
                    %y_j
                    if n == jC && iC~=jC
                        tA = [tA 1];
                    %If iC == jC or n~=iC,jC
                    else
                        tA = [tA 0];
                    end
                end
            end
           fC = [fC; tA]; 
        end
    end
end


%% ------------Linear Equality Constraints-------------
%Creates all necessary equality constraints
function beq = calcBeq(T)
    beq = [];
    %% Z Constraints
    %z1: sum_j(z_ij)=1 i = {2,..., T}
    for i = 2:T
        beq = [beq; 1];
    end
    %z2: sum_i(z_ij)=1 j = {1,...T-1}
    for i = 1:T-1
        beq = [beq; 1];
    end
    %z3: sum_i,j(z_ij)=T-1
    beq = [beq; T-1];
    %z4: sum_i(z_ii)=0
    beq = [beq; 0];

    %% y Constraints
    %yS: y_S = 1
    beq = [beq; 1];

end

% Creates the Equality Matrix
function Aeq = calcAeq(T)
   Aeq = [];
   
   %% Z Constraints
   %z1: for all i sum_j(z_ij)=1
   Aeq = [Aeq; getZ1(T)];
   %z2: sum_i(z_ij)=1 j = {1,...T-1}
   Aeq = [Aeq; getZ2(T)];
   %z3: sum_i,j(z_ij)=T-1
   Aeq = [Aeq; getZ3(T)];
   %z4: sum_i(z_ii)=0
   Aeq = [Aeq; getZ4(T)];

   %% y Constraints
   %yS: y_S = 1
   Aeq = [Aeq; getYs(T)];
   
end


%yS: y_S = 1
function yS = getYs(T)
    %Fills out z
    yS = zeros(1,T^2);
    %Gets y
    for n = 1:T
        if n == 1
            yS = [yS 1];
        else
            yS = [yS 0];
        end
    end
end
%z4: sum_i(z_ii)=0
function z4 = getZ4(T)
    z4 = [];
    %Builds the Diagonal
    for j = 1:T
        for i = 1:T
            if i==j
                z4 = [z4 1];
            else
                z4 = [z4 0];
            end
        end
    end

    %Adds an empty y
    z4 = [z4 zeros(1,T)];
end

%z3: sum_i,j(z_ij)=T
function z3 = getZ3(T)
    z3 = [ones(1,T^2) zeros(1,T)];
end

%z2: sum_i(z_ij)=1 j = {1,...T-1}
function z2 = getZ2(T)
    z2 = []; %Stores data
    %for all i
    for jA=1:T-1
        tA = [];%temporary array
        %Creates vector
        for j = 1:T
            for i = 1:T
                if jA == j
                    tA = [tA 1];
                else
                    tA = [tA 0];
                end
            end
        end
        %Creates an empty y 
        tA = [tA zeros(1,T)];
        %Stores data
        z2 = [z2; tA];
    end
end

%z1: sum_j(z_ij)=1 i = {2,..., T}
function z1 = getZ1(T)
    z1 = []; %Stores data
    %i = {2,..., T}
    for iA=2:T
        tA = [];%temporary array
        %Creates vector
        for j = 1:T
            for i = 1:T
                if iA == i
                    tA = [tA 1];
                else
                    tA = [tA 0];
                end
            end
        end
        %Creates an empty y 
        tA = [tA zeros(1,T)];
        %Stores data
        z1 = [z1; tA];
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
    cV = [cV zeros(1,T)];
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
