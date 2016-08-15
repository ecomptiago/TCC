classdef FuzzyRule
    
    properties
        rule
        rulesIntervals
    end
    
    methods
        %Constructor
        function fuzzyRule = FuzzyRule(rule,input) 
            fuzzyRule.rule = rule;
            rows = size(input);
            rows = rows(2);
            mfSize = size(input(1).mf);
            mfSize = mfSize(2);
            paramsSize = size(input(1).mf(1).params);
            paramsSize = paramsSize(2);
            columms = mfSize * (paramsSize -1);
            fuzzyRule.rulesIntervals = zeros(rows,columms);
            for i=1.0:rows
                for j=0.0:(columms / 2) -1  
                    fuzzyRule.rulesIntervals(i,(j * 2) + 1) = ...
                        input(i).mf(j + 1).params(1);
                    fuzzyRule.rulesIntervals(i,((j * 2) + 1) + 1) = ...
                        input(i).mf(j + 1).params(3);
                end
            end
        end
        
        function activated = wasActivated(instance,meanValues)
            accumulator = true;
            for i = 1:size(meanValues)
                if(instance.rule(i) ~= 0)
                    switch(avoid_obstacles.src.enumerations...
                            .InputQuantifier(instance.rule(i)))
                            case avoid_obstacles.src.enumerations...
                                .InputQuantifier.MUITO_PERTO
                                    accumulator = accumulator & (meanValues(i) >= ...
                                        instance.rulesIntervals...
                                        (i,1) && meanValues(i) <= instance...
                                        .rulesIntervals(i,2));
                            case avoid_obstacles.src.enumerations...
                                .InputQuantifier.PERTO
                                    accumulator = accumulator & (meanValues(i) >= ...
                                        instance.rulesIntervals...
                                        (i,3) && meanValues(i) <= instance...
                                        .rulesIntervals(i,4));
                            case avoid_obstacles.src.enumerations...
                                .InputQuantifier.LONGE
                                    accumulator = accumulator & (meanValues(i) >= ...
                                        instance.rulesIntervals...
                                        (i,5) && meanValues(i) <= instance...
                                        .rulesIntervals(i,6));
                            otherwise
                                fprintf('Error');
                    end
                end
            end
            activated = accumulator;
        end
    end
    
end

