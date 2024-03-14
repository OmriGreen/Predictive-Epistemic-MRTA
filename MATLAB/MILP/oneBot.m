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
T = 1; % Number of Tasks

% Cost Vector
cV = costVector(T, startLoc, endLoc, S);
f = transpose(cV);

% Linear Equality Constraint Equation
Aeq = calcA(T);
beq = calcB(T);

% Integer Variables
intcon = 1:length(cV);

% Lower bound constraint (NO NEGATIVE NUMBERS)
lb = zeros(length(cV),1);

% Upper Bound value
ub = ones(length(cV), 1) * inf;
ub((T+1)^2+1) = T+1; % Adjust upper bound for the last variable

% Inequality Constraint Equation
A = [];
b = [];

% Adjust dimensions of f to match Aeq
f = [f; zeros(size(Aeq, 1) - length(f), 1)];

% Solve MILP with updated options
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



%TEMPORARY TESTING
% for z=2:T+1
%     rawA6 = req6(T,z)
%     A6 = [];
%     t6 = [];
%     for i=1:length(rawA6)
%         t6 = [t6 rawA6(i)];
%         if rem(length(t6),T+2)==0
%             A6 = [A6; t6];
%             t6 = [];
%         end
%     end
%     A6
% end


% ------------Linear Equality Constraints-------------
%Creates all necessary inequality constraints
function b = calcB(T)
    b = [];

    % A1: For all i z_iS=0
    b = [b;0];

    % A2: for all j z_Ej = 0
    b = [b;0];

    % A3: for all j sum_i!=E(z_ij)=1
    for i=1:T+1
        b = [b;1];
    end

    % A4: for all i sum_j!=S(z_ij)=1
    for i=1:T+1
       b = [b;-1];
    end

    % A5: z_SE=0
    %b = [b;0];

    % A6: for all i~=S sum_j~=E(z_ij)-sum_j~=E(z_ji)=0 Eq(4)
    for i=1:T
       %b = [b;0];
    end
    
end

% Creates the inequality Matrix
function A = calcA(T)
    % Calculates all rows of A for the given number of Tasks
    % For all i z_iS=0
    A1 = req1(T);

    % for all j z_Ej = 0
    A2 = req2(T);

    % for all j sum_i!=E(z_ij)=1
    A3 = req3(T);
        
    % for all i sum_j!=S(z_ij)=1
    A4 = req4(T);

    %z_SE=0
    A5 = req5(T); 

    % A6: for all i~=S sum_j~=E(z_ij)-sum_j~=E(z_ji)=0 Eq(4)
    d = req6(T) %Causing conflict
    A6 = [];
    A = [A1;A2;A3;A4; A5; A6];
end


% A6: for all i~=S j~=E z_ij-z_ji=0 Eq(4)
function A6 = req6(T)
    A6 = [];
    %Goes through all potential j which are not in End
    for jC=1:T+1
        %Goes through all potential i which are not in Start
        for iC = 2:T+2
            tA = []; %Temporary array
            %Goes Through all potential j values
            for j=1:T+2
                %Goes Through all possible i values
                for i = 1:T+2
                    if i==iC && j==jC
                        %jumping to
                        tA=[tA 1];
                    else 
                        if j==iC
                            tA = [tA -1];
                        else
                            tA = [tA 0];
                        end
                     end
                end
            end
            %Adds last 0s to constraint
            for z=1:T+2
                tA = [tA 0];
            end 
            A6 = [A6; tA];
        end
    end
end

%z_SE = 0
function A5 = req5(T)
    A5 = [];
    tA = [];
    for i=1:T+2
        for j=1:T+2
            if i == 1 && j==T+2
                tA = [tA 1];
            else
                tA = [tA 0];
            end
        end
    end
    for z=1:T+2
        tA = [tA 0];
    end 
    A5 = [A5;tA];
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
                    else
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
                if j==i && k~=1
                    tA = [tA 1];
                else
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

