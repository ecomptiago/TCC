function [ ] = setaVelocidadeEsquerda( clientID, vrep, motorEsquerdoHandle,  velocidade)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    vrep.simxSetJointTargetVelocity(clientID, motorEsquerdoHandle,-velocidade,vrep.simx_opmode_oneshot);
end

