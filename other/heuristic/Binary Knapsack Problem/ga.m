clc;
clear;
close all;

%% Problem Definition

global NFE;
NFE=0;

model=CreateModel();        % Create Knapsack Model

CostFunction=@(x) KnapsackCost(x,model);    % Cost Function

nVar=model.n;        % Number of Decision Variables

VarSize=[1 nVar];   % Size of Decision Variables Matrix


%% GA Parameters

MaxIt=200;          % Maximum Number of Iterations

nPop=50;            % Population Size

pc=0.8;                 % Crossover Percentage
nc=2*round(pc*nPop/2);  % Number of Offsprings (Parents)

pm=0.3;                 % Mutation Percentage
nm=round(pm*nPop);      % Number of Mutants

mu=0.02;                % Mutation Rate

ANSWER=questdlg('Select the Parent Selection Method:','GA','Random','RWS','TS','RWS');
UseRandomSelection=strcmpi(ANSWER,'Random');
UseRWS=strcmpi(ANSWER,'RWS');
UseTS=strcmpi(ANSWER,'TS');

if UseRWS
    beta=10;                % Selection Pressure
end

if UseTS
    TournamentSize=3;       % Tournament Size
end

pause(0.1);

%% Initialization

% Create Empty Structure
empty_individual.Position=[];
empty_individual.Cost=[];
empty_individual.Sol=[];

% Create Population Matrix (Array)
pop=repmat(empty_individual,nPop,1);

% Initialize Population
for i=1:nPop
    
    % Initialize Position
    pop(i).Position=CreateRandomSolution(model);
    
    % Evaluation
    [pop(i).Cost pop(i).Sol]=CostFunction(pop(i).Position);
    
end

% Sort Population
Costs=[pop.Cost];
[Costs SortOrder]=sort(Costs);
pop=pop(SortOrder);

% Update Best Solution Ever Found
BestSol=pop(1);

% Update Worst Cost
WorstCost=max(Costs);

% Array to Hold Best Cost Values
BestCost=zeros(MaxIt,1);

% Array to Hold NFEs
nfe=zeros(MaxIt,1);


%% GA Main Loop

for it=1:MaxIt
    
    if UseRWS
        % Calculate Selection Probabilities
        P=exp(-beta*Costs/WorstCost);
        P=P/sum(P);
    end
    
    % Crossover
    popc=repmat(empty_individual,nc/2,2);
    for k=1:nc/2
        
        % Select Parents

        if UseRandomSelection
            i1=randi([1 nPop]);
            i2=randi([1 nPop]);
            p1=pop(i1);
            p2=pop(i2);
        end
        
        if UseRWS
            i1=RouletteWheelSelection(P);
            i2=RouletteWheelSelection(P);
            p1=pop(i1);
            p2=pop(i2);
        end
        
        if UseTS
            p1=TournamentSelection(pop,TournamentSize);
            p2=TournamentSelection(pop,TournamentSize);
        end
        
        % Apply Crossover
        [popc(k,1).Position popc(k,2).Position]=BinaryCrossover(p1.Position,p2.Position);
        
        % Evaluate Offsprings
        [popc(k,1).Cost popc(k,1).Sol]=CostFunction(popc(k,1).Position);
        [popc(k,2).Cost popc(k,2).Sol]=CostFunction(popc(k,2).Position);
        
    end
    popc=popc(:);
    
    % Mutation
    popm=repmat(empty_individual,nm,1);
    for k=1:nm
        
        % Select Parent Index
        i=randi([1 nPop]);
        
        % Select Parent
        p=pop(i);
        
        % Apply Mutation
        popm(k).Position=Mutate(p.Position,mu);
        
        % Evaluate Mutant
        [popm(k).Cost popm(k).Sol]=CostFunction(popm(k).Position);
        
    end
    
    % Merge Population
    pop=[pop
         popc
         popm];
     
    % Sort Population
    Costs=[pop.Cost];
    [Costs SortOrder]=sort(Costs);
    pop=pop(SortOrder);
    
    % Truancate Extra Memebrs
    pop=pop(1:nPop);
    Costs=Costs(1:nPop);
    
    % Update Best Solution Ever Found
    BestSol=pop(1);
    
    % Update Worst Cost
    WorstCost=max(WorstCost,max(Costs));
    
    % Update Best Cost Ever Found
    BestCost(it)=BestSol.Cost;
    
    % Update NFE
    nfe(it)=NFE;
    
    % Show Iteration Information
    if BestSol.Sol.IsFeasible
        Flag=' *';
    else
        Flag='';
    end
    
    disp(['Iteration ' num2str(it) ': NFE = ' num2str(nfe(it)) ', Best Cost = ' num2str(BestCost(it)) Flag]);
    
end

%% Results

figure;
plot(nfe,BestCost,'LineWidth',2);
xlabel('NFE');
ylabel('Best Cost');

