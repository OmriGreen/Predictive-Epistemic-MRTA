% clear, pack, clc
% References:
% https://aleksandarhaber.com/solve-mixed-integer-linear-programming-milp-optimization-problems-in-matlab/,
% https://www.mathworks.com/help/optim/ug/travelling-salesman-problem.html
%https://www.sciencedirect.com/science/article/pii/S1366554517302880#e0255   
    %Eq 4 Ensures continuous Graph

clear, clc
% Scenario Setup
startLoc = [0 0];
endLoc = [0 0];
T = 2;
cV = costVector(T, startLoc, endLoc);


% Calculates all rows of A for the given number of Tasks
A1 = req1(T); %For all i z_iS=0
A2 = req2(T); %for all j z_Ej = 0
A3 = req3(T); %for all j sum_i!=E(z_ij)=1
A4 = req4(T); %for all i sum_j!=S(z_ij)=1
A5 = req5(T); %for all ij z_ij -> y_i = y_j+1 (y_S=T+1, y_E = 0)


%for all ij z_ij -> y_i = y_j+1 (y_S=T+1, y_E = 0)
function A5 = req5(T)
     %Inputs
    % T: The Number of Tasks
    %Outputs
    %A5: A set of 3 arrays showing for all ij z_ij -> y_i = y_j+1
    A5 = [];
    tA = [];

    %y_S = T+1
    for i=1:T+2
        %Goes Through the j values
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
    
    %y_E = 0
    tA = [];
    for i=1:T+2
        %Goes Through the j values
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


end
%for all i sum_j!=S(z_ij)=1
function A4 = req4(T)
  %Inputs
    % T: The Number of Tasks
    %Outputs
    %A4: A set of T+1 arrays showing for all i sum_j!=S(z_ij)=1
    A4 = [];
     %Goes through the process T+1 times
    for i=1:T+2
        if i~= 1
            tA = []; %Temporary requirement for each line
            %Goes Through the i values
            for j=1:T+2
                %Goes Through the j values
                for k=1:T+2
                    if k==i
                        tA = [tA 1];
                    end
                    if k~=i
                        tA = [tA 0];
                    end
                end
            end
            %Adds values for y (irrelevant in this requirement)
            for z= 1:T+2
                tA =[tA 0];
            end 
            A4 = [A4;tA];
        end
    end
end

%for all j sum_i!=E(z_ij)=1
function A3 = req3(T)
     %Inputs
    % T: The Number of Tasks
    %Outputs
    %A3: A set of T+1 arrays showing for all i sum_j!=E(z_ij)=1

    A3 = [];
    %Goes through the process T+1 times
    for i=1:T+1
        tA = []; %Temporary requirement for each line
        %Goes Through the i values
        for j=1:T+2
            %Goes Through the j values
            for k=1:T+2
                if j==i
                    tA = [tA 1];
                end
                if j~=i
                    tA = [tA 0];
                end
            end
        end
        %Adds values for y (irrelevant in this requirement)
        for z= 1:T+2
            tA =[tA 0];
        end 
        A3 = [A3;tA];
    end
   
end

%for all j z_Ej = 0
function A2 = req2(T)
    %Inputs
    % T: The Number of Tasks
    %Outputs
    %A2: An array listing where j = S
    
    A2 = [];
    %Sorts through all real values
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
   %Adds values for y (irrelevant in this requirement)
    for i= 1:T+2
        A2=[A2 0];
    end 
end

%For all i z_iS=0
function A1 = req1(T)
    %Inputs
    % T: The Number of Tasks
    %Outputs
    %A1: An array listing where i = S
    %b1: 0
    
    
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
    
     %Adds values for y (irrelevant in this requirement)
    for i= 1:T+2
        A1=[A1 0];
    end 
end

%% ---------Cost Vector Functions----------

%Creates a cost Vector for a MILP problem
function cV = costVector(T,startLoc,endLoc)
    %T: Number of Tasks
    %startLoc: Starting Location
    %endLoc: Ending Location
    cV = [];
    %Generates random locations
    locations = setup(T, startLoc, endLoc);
    
    %Finds distances between all vectors and adds them to the cost vector
    for i=1:T+2
        for j=1:T+2
            x1=locations(i,1);
            x2=locations(j,1);
            y1=locations(i,2);
            y2=locations(j,2);
            cV=[cV findDist(x1,x2,y1,y2)];
        end
    end
    %Adds a 0 to the last column of data for the cost vector to make it work
    %with y later in the code
    for i=1:T+2
        cV = [cV 0];
    end
end

%Finds the distance between two points
function distance = findDist(x1,x2,y1,y2)
    distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end

%Creates Locations for all tasks with a set amount of tasks
function taskLocations = setup(T, startLoc, endLoc)
    %T is the number of tasks
    maxSize = 50; %Maximum Size of an nxn area
    taskLocations = []; %Stores all the Tasks in a array
    taskLocations = [taskLocations; startLoc];
    for i=1:T
        a = floor(maxSize*rand);
        b = floor(maxSize*rand);
        taskLocations = [taskLocations; [a,b]];
    end
    taskLocations = [taskLocations;endLoc];
end

