function [y1 y2]=BinaryCrossover(x1,x2)

    M=randi([1 3]);
    
    switch M
        case 1
            % Single Point Crossover
            [y1 y2]=SinglePointCrossover(x1,x2);
            
        case 2
            % Double Point Crossover
            [y1 y2]=DoublePointCrossover(x1,x2);
            
        case 3
            % Uniform Crossover
            [y1 y2]=UniformCrossover(x1,x2);
            
    end

end