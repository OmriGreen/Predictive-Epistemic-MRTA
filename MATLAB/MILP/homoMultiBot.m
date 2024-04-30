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

%Gets all constraints and locations
[S,cV,locs,f,Aeq,beq,intcon,lb,ub,A,b,T,K] = buildConstraints();

%Solves MILP Classically
MILPClassic(f,intcon,A,b,Aeq,beq,lb,ub,locs,T,K,S);


%% Solve MILP classically
function MILPClassic(f,intcon,A,b,Aeq,beq,lb,ub,locs,T,K,S)
    rawX = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub);
    print(rawX,T,K);
    visualizeData(locs,rawX,T,K,S);
end



%% Visualizes Data
function visualizeData(locs,sol,T,K,S)
    %Gets all x and y values
    x = transpose(locs(:,1));
    y = transpose(locs(:,2));
    
    %Generates all edges from a solution for individual robots per row
    tempEdges = [];

    for k = 1:K
        tE = [];
        for j = 1:T
            for i = 1:T
                pos = i + T*(j-1) +(k-1)*(T^2+T);
                
                %Creates an edge if there is a value there
                if sol(pos) == 1
                    e1 = locs(j,:);
                    e2 = locs(i,:);
                    %finds the time
                    timePos = i+T^2 + (k-1)*(T^2+T);
                    e3 = sol(timePos);
                    tE = [tE transpose([e1 e2 e3])];

                end
            end
        end
        tempEdges = [tempEdges; tE];
    end
    
    allVals = sum(sol);

    if allVals > 1
        %Orders Tasks per robot
        edges = [];
        for k = 1:K
            subE = [];
            %Gets a subset of tasks for a robot
            
            subset = tempEdges((k-1)*5+1:k*5,:);
            %Sorts the subset
            z = size(subset);
            z = z(2);
            for iC = 2:T
                tE = zeros(4,1);
                for i = 1:z
                   if subset(5,i)==iC
                       tE = subset(1:4,i);
                   end
                end
                subE = [subE tE];
            end
            edges = [edges; subE];
        end
    end
    
    %initially graphs the locations on the map
    x = [];
    y = [];
    
    numLocs = size(locs);
    numLocs = numLocs(1);

    for i = 1:numLocs
        x = [x locs(i,1)];
        y = [y locs(i,2)];
    end
    
    scatter(x,y,'filled');
    text(x, y, cellstr(num2str((1:numel(x))')),'VerticalAlignment','bottom','HorizontalAlignment','right');
    title(['TSP with ' num2str(T) ' tasks & ' num2str(K) ' robots']);
    xlabel('X coordinate');
    ylabel('Y coordinate');
    grid on;

    %Set min and max positions
    xlim([0,S]);
    ylim([0,S]);

    if allVals > 1
        %Draws the path each robot takes
        for k = 1:K
            x = [];
            y = [];
            subset = edges((k-1)*4+1:k*4,:);
            for i = 1:T-1
                tEdge = subset(:,i);
                tEdge = transpose(tEdge);
                total = sum(tEdge);
                if total ~= 0
                    x = [x tEdge(1) tEdge(3)];
                    y = [y tEdge(2) tEdge(4)];
                end
    
            end
            %Draws the arrow
            line(x,y);
    
        end
    end



end


%% Builds all data
function [S,cV,locs,f,Aeq,beq,intcon,lb,ub,A,b,T,K] = buildConstraints()

%Sets up the scenario
S = input('Enter the size in a nxn grid = ');
T = input('Enter the number of tasks = ');
K = input('Enter the number of robots = ');
% Cost Vector
[cV,locs] = costVector(T, S,K);
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

end



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
%for all i,j,z min(z_ijz) = 0, min(y_i)=0
function lb = calcLB(T,K)
    lb = zeros(1,(T+T^2)*K);
end

% Upper Bound
%for all i,j,z max(z_ijz) = 1, max(y_i)=T
function ub = calcUB(T,K)
    ub = [];
    temp = [ones(1,T^2) T*ones(1,T)];
    for i = 1:K
        ub = [ub temp];
    end
end


%% ======= Linear Inequality Constraints ======
%Calculate Inequality Constraints
function bI=calcBIeq(T,K)
    bI = []; 
end

%Calculate the Inequality Matrix
function I=calcIeq(T,K)
   I=[];
end

%% ===Linear Equality Constraints===
%Creates all necessary equality constraints
function beq = calcBeq(T,K)
    beq = [];
end

% Creates the Equality Matrix
function Aeq = calcAeq(T,K)
   Aeq = [];
end

%% ---------Cost Vector Functions----------

%Creates a cost Vector for a MILP problem
function [cV,locations] = costVector(T, S,K)
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
        x = floor(maxSize*rand);
        y = floor(maxSize*rand);
        taskLocations = [taskLocations; [x,y]];
    end
end