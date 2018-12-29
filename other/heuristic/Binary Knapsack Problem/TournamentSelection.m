function p=TournamentSelection(pop,m)

    n=numel(pop);
    
    A=randsample(n,m);
    
    spop=pop(A);
    
    costs=[spop.Cost];
    [~, i]=min(costs);
    
    p=spop(i);

end