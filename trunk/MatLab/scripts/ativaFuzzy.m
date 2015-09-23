function [ saida ] = ativaFuzzy( LaserValues )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    saida=0;
    for i=1:size(LaserValues,2)
        if(LaserValues(1,i) < 0.5)
           saida=1;
           break;
        end
    end
end

