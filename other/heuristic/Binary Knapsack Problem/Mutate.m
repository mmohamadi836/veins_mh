function y=Mutate(x,mu)

    nVar=numel(x);
    
    nMu=ceil(mu*nVar);
    
    j=randsample(nVar,nMu);
    
    y=x;
    
    y(j)=1-x(j);

end