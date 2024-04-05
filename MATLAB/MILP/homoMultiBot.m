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
K = 5; % Number of Robots

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
%Prints out a 2d representation of 

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
        %y_S
        lb = [lb 1];
        %y_n >= 0, n = {2,...,T-1}
        for n = 2:T-1
            lb = [lb 0];
        end
        %y_E >= 2
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
            ub = [ub T-1];
        end
        %y_E <=T
        ub = [ub T];
    end
end
%% ======= Linear Inequality Constraints ======
%Calculate Inequality Constraints
function bI=calcBIeq(T,K)
    bI = [];   
    %for all k, i={2,...,T-1}, j = {1,...,T-1}
    %-sum_j(sum_i(z_ijk))<=-(T-2)/K+1  
    for i=1:K
        bI=[bI; -(T-2)/K+1];
    end
    %for all k, i={2,...,T-1}, j = {1,...,T-1}
    %sum_j(sum_i(z_ijk))<=(T-2)/K+1  
    for i=1:K
        bI=[bI; (T-2)/K+1];
    end

end

%Calculate the Inequality Constraints
function I=calcIeq(T,K)
   I=[];
   %% Z Constraints
   %for all k, i={2,...,T-1}, j = {1,...,T-1}
   %-sum_j(sum_i(z_ijk))<=-(T-2)/K+1  
   I = [I; zL(T,K)];
   %for all k, i={2,...,T-1}, j = {1,...,T-1}
   %sum_j(sum_i(z_ijk))<=(T-2)/K+1  
   I = [I; zG(T,K)];
   %% Flow Constraint
end

%for all k, i={2,...,T-1}, j = {1,...,T-1}
%sum_j(sum_i(z_ijk))<=(T-2)/K+1  
function z = zG(T,K)
    z = [];
    %Chooses what k will be analyzed
    for r = 1:K
        zR = [];
        %Goes Through all robots
        for k = 1:K
            zK = [];
            %Goes Through all z
            for j = 1:T
                zJ = [];
                for i = 1:T
                    if r==k && j<T && i>1 && i < T
                        zJ = [zJ 1];
                    else
                        zJ = [zJ 0];
                    end
                end
                zK = [zK zJ];
            end
            zK = [zK zeros(1,T)];
            zR = [zR zK];
        end
        z = [z; zR];
    end
end

%for all k, i={2,...,T-1}, j = {1,...,T-1}
   %-sum_j(sum_i(z_ijk))<=-(T-2)/K+1  
function z = zL(T,K)
    z = [];
    %Chooses what k will be analyzed
    for r = 1:K
        zR = [];
        %Goes Through all robots
        for k = 1:K
            zK = [];
            %Goes Through all z
            for j = 1:T
                zJ = [];
                for i = 1:T
                    if r==k && j<T && i>1 && i < T
                        zJ = [zJ -1];
                    else
                        zJ = [zJ 0];
                    end
                end
                zK = [zK zJ];
            end
            zK = [zK zeros(1,T)];
            zR = [zR zK];
        end
        z = [z; zR];
    end
end


%% ===Linear Equality Constraints===
%Creates all necessary equality constraints
function beq = calcBeq(T,K)
    beq = [];
    %% Z Constraints
    %sum_k(sum_j(z_Sjk))=0
    beq = [beq; 0];
    %for all k sum_j(z_Ejk)=1
    for i=1:K
        beq = [beq; 1];
    end
    %i={2,...,T-1} sum_k(sum_j(z_ijk))=1
    for i=2:T-1
        beq = [beq; 1];
    end
    %sum_k(sum_i(z_iik))=0
    beq = [beq; 0];
    %for all k sum_i(z_iSk) = 1
    for i=1:K
        beq = [beq; 1];
    end
    %j={2,...,T-1} sum_k(sum_i(z_ijk))=1
    for i=2:T-1
        beq = [beq; 1];
    end

    %% y constraints
    %for all k y_Sk=1
    for i=1:K
        beq = [beq; 1];
    end
    %for all k y_E-sum_i(sum_j(z_ijk))=1
    for i=1:K
        beq = [beq; 1];
    end
end

% Creates the Equality Matrix
function Aeq = calcAeq(T,K)
   Aeq = [];

   %% Z Constraints
   %sum_k(sum_j(z_Sjk))=0
   Aeq = [Aeq; z_S(T,K)];
   %for all k sum_j(z_Ejk)=1
   Aeq = [Aeq; z_E(T,K)];
   %i={2,...,T-1} sum_k(sum_j(z_ijk))=1
   Aeq = [Aeq; z_i(T,K)];
   %sum_k(sum_i(z_iik))=0
   Aeq = [Aeq; z_ii(T,K)];
   %for all k sum_i(z_iSk) = 1
   Aeq = [Aeq; z_iSk(T,K)];
   %j={2,...,T-1} sum_k(sum_i(z_ijk))=1
   Aeq = [Aeq; z_j(T,K)];
   %% y constraints
   %for all k y_Sk=1
   Aeq = [Aeq; y_Sk(T,K)];
   %for all k y_E-sum_i(sum_j(z_ijk))=1
   Aeq = [Aeq; y_Ek(T,K)];
end

%for all k y_E-sum_i(sum_j(z_ijk))=1
function y=y_Ek(T,K)
    y=[];
    %For all seperate robots
    for r=1:K
        yR = [];
    %Goes Through every robot
        for k=1:K
            yK=[];
            %Enters in -1 if k==r for all z otherwise it enters 0
            if k==r
                yR=[yR -ones(1,T^2)];
            else
                yR=[yR zeros(1,T^2)];
            end
            
            %Enters in y_Sk=1 when k==r
            for n=1:T
                if r==k && n==T
                    yR=[yR 1];
                else
                    yR=[yR 0];
                end
            end
            
        end
        y=[y;yR];

    end
end


%for all k y_Sk=1
function y=y_Sk(T,K)
    y=[];
    %For all seperate robots
    for r=1:K
        yR = [];
    %Goes Through every robot
        for k=1:K
            yK=[];
            %Enters in 0 for all z
            yR=[yR zeros(1,T^2)];
            
            %Enters in y_Sk=1 when k==r
            for n=1:T
                if r==k && n==1
                    yR=[yR 1];
                else
                    yR=[yR 0];
                end
            end
            
        end
        y=[y;yR];

    end
end


%j={2,...,T-1} sum_k(sum_i(z_ijk))=1
function z = z_j(T,K)
    z = [];
    %For all j levels
    for jC=2:T-1
        zR = [];
    %Goes Through every robot
        for k=1:K
            zK=[];
            %Goes Through z for robot# k
            for j=1:T
              zJK = [];
              for i = 1:T
                   if j == jC
                       zJK=[zJK 1];
                   else
                       zJK=[zJK 0];
                   end
              end
              zK = [zK zJK];
              
            end
            zR=[zR zK];
            zR = [zR zeros(1,T)];
            
        end
        z=[z;zR];

    end
end

%for all k sum_i(z_iSk) = 1
function z = z_iSk(T,K)
    z = [];
    %For all seperate robots
    for r=1:K
        zR = [];
    %Goes Through every robot
        for k=1:K
            zK=[];
            %Goes Through z for robot# k
            for j=1:T
              zJK = [];
              for i = 1:T
                   if j == 1 && r==k
                       zJK=[zJK 1];
                   else
                       zJK=[zJK 0];
                   end
              end
              zK = [zK zJK];
              
            end
            zR=[zR zK];
            zR = [zR zeros(1,T)];
            
        end
        z=[z;zR];

    end
end

%sum_k(sum_i(z_iik))=0
function z = z_ii(T,K)
    z = [];
    zT = [];
    for j = 1:T
        for i = 1:T
            if i==j
                zT = [zT 1];
            else
                zT = [zT 0];
            end
        end
    end
    zT = [zT zeros(1,T)];
    
    for k = 1:K
        z = [z zT];
    end
    

end


%i={2,...,T-1} sum_j(z_ijk))=1
function z=z_i(T,K)
    z = [];
    %Goes through all columns except the end and start
    for c=2:T-1
        zC = [];
        %Goes Through Every Robot
        for k=1:K
            zK = [];
            %Goes Through z for robot # k
            for j=1:T
                zJC = [];
                for i = 1:T
                    if i==c
                        zJC = [zJC 1];
                    else
                        zJC = [zJC 0];
                    end
                end
                zK = [zK zJC];
            end
            zC = [zC zK];
            zC = [zC zeros(1,T)];
        end
        z = [z; zC];
    end
end
%for all k sum_j(z_Ejk)=1
function z=z_E(T,K)
    z = [];
    %For all seperate robots
    for r=1:K
        zR = [];
    %Goes Through every robot
        for k=1:K
            zK=[];
            %Goes Through z for robot# k
            for j=1:T
              zJK = [];
              for i = 1:T
                   if i == T && r==k
                       zJK=[zJK 1];
                   else
                       zJK=[zJK 0];
                   end
              end
              zK = [zK zJK];
              
            end
            zR=[zR zK];
            zR = [zR zeros(1,T)];
        end
        z=[z;zR];

    end
end
%sum_k(sum_j(z_Sjk))=0
function z=z_S(T,K)
  z=[];
  
  %Goes Through every robot
  for k=1:K
      zK=[];
      %Goes Through z for robot# k
      for j=1:T
          zJK = [];
          for i = 1:T
               if i == 1
                   zJK=[zJK 1];
               else
                   zJK=[zJK 0];
               end
          end
          zK = [zK zJK];
          
      end
      z=[z zK];
      z = [z zeros(1,T)];
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