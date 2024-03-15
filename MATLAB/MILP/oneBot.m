% clear, pack, clc
% References:
% https://aleksandarhaber.com/solve-mixed-integer-linear-programming-milp-optimization-problems-in-matlab/,
% https://www.mathworks.com/help/optim/ug/travelling-salesman-problem.html
%https://www.sciencedirect.com/science/article/pii/S1366554517302880#e0255   
    %Eq 4 Ensures continuous Graph

clear, clc

% Scenario Setup
S = 50; % The max X and max Y positions
startLoc = [0 0]; % Start Location [x y]
endLoc = [0 0]; % End Location [x y]
T = 4; % Number of Tasks

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
A = calcIeq(T);
b = calcBIeq(T);

% Adjust dimensions of f to match Aeq
f = [f; zeros(size(Aeq, 1) - length(f), 1)];

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

%% ======= Linear Inequality Constraints ======
%Calculate Equality Constraints
function bI=calcBIeq(T)
    bI = [];

    %for all n!=E y_n>=1
    for i=1:T+1
        bI=[bI;-1];
    end
    
    %for all n!=S y_n<=T+1
    for i=1:T+1
        bI=[bI;(T)];
    end
end

%Calculate the Inequality Matrix
function I=calcIeq(T)
    I = [];

    %for all n!=E y_n>=1
    I1 = Ieq1(T);
    
    %for all n!=E y_n<=T+1
    I2 = Ieq2(T);

    I = [I1; I2];
end

%for all n!=E y_n<=T+1
function I2 = Ieq2(T)
    I2 = [];
    for z = 2:T+2
        tA = [];
        for i=1:T+2
            for j=1:T+2
                tA=[tA 0];
            end
        end
        
        for i=1:T+2
            if i==z
                tA=[tA 1];
            else
                tA=[tA 0];
            end
        end
        I2 = [I2; tA];
    end
end

%for all n!=E y_n>=1
function I1 = Ieq1(T)
    I1 = [];
    for z = 1:T+1
        tA = [];
        for i=1:T+2
            for j=1:T+2
                tA=[tA 0];
            end
        end
        
        for i=1:T+2
            if i==z
                tA=[tA -1];
            else
                tA=[tA 0];
            end
        end
        I1 = [I1; tA];
    end
end


%% ------------Linear Equality Constraints-------------
%Creates all necessary equality constraints
function beq = calcBeq(T)
    beq = [];

    % A1: For all i z_iS=0
    beq = [beq;0];

    % A2: for all j z_Ej = 0
    beq = [beq;0];

    % A3: for all j sum_i!=E(z_ij)=1
    for i=1:T+1
        beq = [beq;1];
    end

    % A4: for all i sum_j!=S(z_ij)=1
    for i=1:T+1
        beq = [beq;1];
    end

    % A5: y_S=T+1, y_E = 0
    beq=[beq;T+1];
    beq=[beq;0];
    %sum_n(y_n)=T+1 + T + ... + 1
    sum = 0;
    for i=1:T+1
        sum = sum+i;
    end
    
    beq =[beq;sum];
    
    %A6: z_ES=0
    beq =[beq;0];

    %A7: for all i!=E z_ii-0
    beq = [beq;0];

    %A8: for all j sum_i!=S(z_ij)-sum_i!=S(z_ji)=0

end

% Creates the Equality Matrix
function Aeq = calcAeq(T)
    % Calculates all rows of A for the given number of Tasks
    A1 = req1(T); % For all i z_iS=0
    A2 = req2(T); % for all j z_Ej = 0
    A3 = req3(T); % for all j sum_i!=E(z_ij)=1
    A4 = req4(T); % for all i sum_j!=S(z_ij)=1
    A5 = req5(T); % for all ij z_ij -> y_i = y_j+1 (y_S=T+1, y_E = 0)
    A6 = req6(T); %A6: z_ES=0
    A7 = req7(T); %A7: for all i!=E z_ii=0
    A8 = req8(T) %A8: for all j sum_i!=S(z_ij)-sum_i!=S(z_ji)=0
    Aeq = [A1;A2;A3;A4;A5;A6;A7];
end

%A8: for all j sum_i!=S(z_ij)-sum_i!=S(z_ji)=0
function A8 = req8(T)
    A8 = [];

    for jC = 1:T+1
        tA = [];
        for j=1:T+2
            for i=1:T+2
                if jC == j && i~= 1 && i~= T+2
                    tA = [tA 1];
                else
                    if jC == i && j~= 1 && j~= T+2
                        tA = [tA -1];
                    else
                        tA = [tA 0];
                    end
                end
            end
        end
        for z=1:T+2
            tA = [tA 0];
        end
        A8 = [A8; tA];
    end
end

%A7: for all i!=E z_ii-0
function A7=req7(T)
    A7 = [];
     for j=1:T+2
        for i=1:T+2
            if i==j && j~=T+2 
                A7=[A7 1];
            else
                A7=[A7 0];
            end
            
        end
    end
    for i=1:T+2
        A7=[A7 0];
    end 
end
   


%A6: z_ES = 0
function A6 = req6(T)
    A6 = [];

    for i=1:T+2
        for j=1:T+2
            if j==T+2 && i==1
                A6=[A6 1];
            else
                A6=[A6 0];
            end
        end
    end
    
    for i=1:T+2
        A6=[A6 0];
    end 
end

% for all ij z_ij -> y_i = y_j+1 (y_S=T+1, y_E = 0, sum_n(y_n)=T+1)
function A5 = req5(T)
    A5 = [];
    tA = [];

    for i=1:T+2
        for j=1:T+2
            tA = [tA 0];
        end
    end
    for i=1:T+2
        if i == 1
            tA = [tA 1];
        end
        if i ~= 1
            tA = [tA 0];
        end
    end
    A5 = [A5; tA];
    
    tA = [];
    for i=1:T+2
        for j=1:T+2
            tA = [tA 0];
        end
    end
    for i=1:T+2
        if i == T+2
            tA = [tA 1];
        end
        if i ~= T+2
            tA = [tA 0];
        end
    end
    A5 = [A5; tA];

    tA = [];
    for i=1:T+2
        for j=1:T+2
            tA = [tA 0];
        end
    end
    for i=1:T+2
        tA = [tA 1];
    end
    A5 = [A5; tA];
end

% for all i sum_j!=S(z_ij)=1
function A4 = req4(T)
    A4 = [];

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
            A4 = [A4;tA];
        end
    end
end

% for all j sum_i!=E(z_ij)=1
function A3 = req3(T)
    A3 = [];

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
        A3 = [A3;tA];
    end
end

% for all j z_Ej = 0
function A2 = req2(T)
    A2 = [];

    for i=1:T+2
        for j=1:T+2
            if i==T+2 
                A2=[A2 1];
            end
            if(i~=T+2)
                A2=[A2 0];
            end
        end
    end
    for i=1:T+2
        A2=[A2 0];
    end 
end

% for all i z_iS=0
function A1 = req1(T)
    A1 = [];

    for i=1:T+2
        for j=1:T+2
            if j==1 
                A1=[A1 1];
            end
            if(j~=1)
                A1=[A1 0];
            end
        end
    end
    
    for i=1:T+2
        A1=[A1 0];
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

