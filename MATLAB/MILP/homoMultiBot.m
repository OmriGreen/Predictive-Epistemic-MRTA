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
T = 4; % Number of Tasks
K = 2; % Number of Robots

% Cost Vector
cV = costVector(T, S,K);
f = transpose(cV);

% Linear Equality Constraint Equation
Aeq = calcAeq(T,K);
beq = calcBeq(T,K);

% Integer Variables
intcon = 1:length(cV);

% Lower bound constraint (NO NEGATIVE NUMBERS)
lb = calcLB(T,K);

% Upper Bound value
ub = calcUB(T,K);

% Inequality Constraint Equation
A = calcIeq(T,K);
b = calcBIeq(T,K);

%Solve MILP with updated options
rawX = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub);
print(rawX,T,K);
%% ====== Prints out Arrays ==============
function print(input,T,K)
    z = 0;
    for k=1:K
        k
        x = [];
        for j = 1:T+1
            temp = [];
    
            for i = 1:T
                z=z+1;
                temp = [temp input(z)];
            end
            x = [x;temp];
    
        end

        x
    end
end


%% ======= Bound Calculations =======
%Lower Bound
%for all i,j,z min(z_ijz) = 0, y_S=1, y_i >=2 n = {2,...,T-1}, y_E >= (T-2)/K-1
function lb = calcLB(T,K)
    lb = [];
    for k=1:K
        %Goes Through every Z
        for j=1:T
            for i=1:T
                lb = [lb 0];
            end
        end
        %Creates each Y
        %for all n
        for n = 1:T-1
            lb = [lb 0];
        end
        lb = [lb 2];
    end
end

%Upper Bound
%for all i,j,z min(z_ijz) = 0, y_S=1, y_i >=2 n = {2,...,T-1}, y_E >= (T-2)/K-1
function ub = calcUB(T,K)
    ub = [];
    for k=1:K
        %Goes Through every Z
        for j=1:T
            for i=1:T
                ub = [ub 1];
            end
        end
        %Creates each Y
        %y_S=1
        ub = [ub 1];
        %y_n <= T-1, n = {2,...,T-1}
        for n = 2:T-1
            ub = [ub floor(T/K)];
        end
        %y_E <=T/K+1
        ub = [ub floor(T/K)+1];
    end
end
%% ======= Linear Inequality Constraints ======
%Calculate Inequality Constraints
function bI=calcBIeq(T,K)
    bI = []; 
    for i = 1:K*(T-1)^2
       bI = [bI; floor(T/K)-1];
    end


end

%Calculate the Inequality Matrix
function I=calcIeq(T,K)
   %% Flow Constraint
   I=[];
   I = [I; getFC(T,K)];
end

%y_j-y_i +(T/K-1)z_ij <= T/K-2 i,j={2,...T/K}
function fC = getFC(T,K)
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
                        tA = [tA floor(T/K)];
                       
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
            for kC=1:K
                flow = [];
                for k = 1:K
                    if k==kC
                        flow=[flow tA];
                    else
                        %[r,c] = size(tA);
                        flow=[flow zeros(1,T^2+T)];
                    end
                end
                fC = [fC; flow]; 

            end
        end
    end
end



%% ===Linear Equality Constraints===
%Creates all necessary equality constraints
function beq = calcBeq(T,K)
    beq = [];

   %% y constraints
   %for all k y_Sk=1
   for i = 1:K
       beq = [beq; 1];
   end
   %for all k y_Ek-1 = sum_i(sum_j(z_ijk)) -> y_Ek-1-sum_i(sum_j(z_ijk))=0
   for i = 1:K
       beq = [beq; 1];
   end

   %% z constraints
   % sum_k(sum_i(z_iik))=0
   beq = [beq; 0];
   % sum_k(sum_j(z_Sjk))=0
   beq = [beq; 0];
   % sum_k(sum_i(z_iEk))=0
   beq = [beq; 0];
   %i = [2, ... , T-1] sum_k(sum_j(z_ijk))=1
   for i = 2:T-1
       beq = [beq; 1];
   end
   % for all k sum_j(z_Ejk)=1
   for i = 1:K
       beq = [beq; 1];
   end
   %j=[2,T-1] sum_k(sum_k(z_ijk))=1
   for i = 2:T-1
       beq = [beq; 1];
   end
   %for all k sum_i(z_iSk)=1
   for i = 1:K
       beq = [beq; 1];
   end
end

% Creates the Equality Matrix
function Aeq = calcAeq(T,K)
   Aeq = [];

   %% y constraints
   %for all k y_Sk=1
   Aeq = [Aeq; yE1(T,K)];
   %for all k y_Ek-1 = sum_i(sum_j(z_ijk)) -> y_Ek-1-sum_i(sum_j(z_ijk))=0
   Aeq = [Aeq; yE2(T,K)];
       
   %% z constraints
   % sum_k(sum_i(z_iik))=0
   Aeq = [Aeq; zE1(T,K)];
   % sum_k(sum_j(z_Sjk))=0
   Aeq = [Aeq; zE2(T,K)];
   % sum_k(sum_i(z_iEk))=0
   Aeq = [Aeq; zE3(T,K)];
   %i = [2, ... , T-1] sum_k(sum_j(z_ijk))=1
   Aeq = [Aeq; zE4(T,K)];
   % for all k sum_j(z_Ejk)=1
   Aeq = [Aeq; zE5(T,K)];
   %j=[2,T-1] sum_k(sum_k(z_ijk))=1
   Aeq = [Aeq; zE6(T,K)];
   %for all k sum_i(z_iSk)=1
   Aeq = [Aeq; zE7(T,K)];
end

%for all k sum_i(z_iSk)=1
function z = zE7(T,K)
    z = [];
    for kC=1:K
        zC = [];
        for k = 1:K
            if k == kC
                zC = [zC ones(1,T)];
                zC = [zC zeros(1,T^2)];
            else
                zC = [zC zeros(1,T^2+T)];
            end
        end
        z = [z; zC];
    end
end

%j=[2,T-1] sum_k(sum_k(z_ijk))=1
function z = zE6(T,K)
    z = [];
    for jC = 2:T-1
        zC = [];
        for k = 1:K
            for j = 1:T
                for i = 1:T
                    if j==jC
                        zC = [zC 1];
                    else
                        zC = [zC 0];
                    end
                end
            end
            zC = [zC zeros(1,T)];
        end
        z = [z;zC];
    end
end

% for all k sum_j(z_Ejk)=1
function z = zE5(T,K)
    z = [];
    %Determines which robot is being analyzed
    for kC = 1:K
        zC = [];
        for k = 1:K
            for j = 1:T
                for i = 1:T
                    if i == T && k == kC
                        zC = [zC 1];
                    else
                        zC = [zC 0];
                    end
                end
            end
            zC = [zC zeros(1,T)];
        end
        z = [z; zC];
    end
end

%i = [2, ... , T-1] sum_k(sum_j(z_ijk))=1
function z = zE4(T,K)
    z = [];
    %Determines which column is being analyzed
    for iC = 2:T-1
        zC = [];
        for k = 1:K
            %Goes Through all z
            for j = 1:T
                for i = 1:T
                    if i==iC
                        zC = [zC 1];
                    else
                        zC = [zC 0];
                    end
                end
            end
            zC = [zC zeros(1,T)];
        end
        z = [z;zC];
    end
end

% sum_k(sum_i(z_iEk))=0
function z = zE3(T,K)
    z = [];
    for k = 1:K
        %Goes through all z
        for j = 1:T
            for i = 1:T
                if j==T
                    z = [z 1];
                else
                    z = [z 0];
                end
            end
        end
        %Goes Through all y
        z = [z zeros(1,T)];
    end

end

% sum_k(sum_j(z_Sjk))=0
function z = zE2(T,K)
    z = [];
    for k = 1:K
        %Goes through all z
        for j = 1:T
            for i = 1:T
                if i==1
                    z = [z 1];
                else
                    z = [z 0];
                end
            end
        end
        %Goes Through all y
        z = [z zeros(1,T)];
    end

end

% sum_k(sum_i(z_iik))=0
function z = zE1(T,K)
    z = [];
    %Creates a generic k vector
    z_k = [];
    for j = 1:T
        for i = 1:T
            if i==j
                z_k = [z_k 1];
            else
                z_k = [z_k 0];
            end
        end
    end
    z_k = [z_k zeros(1,T)];

    %Creates the matrix
    for k = 1:K
        z = [z z_k];
    end
end

%for all k y_Ek-1 = sum_i(sum_j(z_ijk)) -> y_Ek-sum_i(sum_j(z_ijk))=1
function y = yE2(T,K)
    y = [];

    %Creates a generic vector to be added
    y_k = [];
    %Fills out z for all k
    y_k = [y_k -ones(1,T^2)];
    %y_Ek = 1
    y_k = [y_k zeros(1,T-1)];
    y_k = [y_k 1];

    %Builds Matrix
    for kC = 1:K
        y_C = [];
        for k = 1:K
            if k == kC
                y_C = [y_C y_k];
            else
                y_C = [y_C zeros(1, T^2+T)];
            end
        end
        y = [y; y_C];
    end

end
%for all k y_Sk=1
function y=yE1(T,K)
    y = [];

    %Creates a generic vector to be added
     y_k = [];
    %Fills out z for all k
    y_k = [y_k zeros(1,T^2)];
    %y_S = 1
    y_k = [y_k 1];
    y_k = [y_k zeros(1,T-1)];

    %Builds Matrix
    for kC = 1:K
        y_C = [];
        for k = 1:K
            if k == kC
                y_C = [y_C y_k];
            else
                y_C = [y_C zeros(1, T^2+T)];
            end
        end
        y = [y; y_C];
    end
    
end


%% ---------Cost Vector Functions----------

%Creates a cost Vector for a MILP problem
function cV = costVector(T, S,K)
    cV = [];
    locations = setup(T, S);
    
    for k=1:K
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