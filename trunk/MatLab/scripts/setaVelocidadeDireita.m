function [ ] = setaVelocidadeDireita( clientID, vrep, motorDireitoHandle,  velocidade)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
    vrep.simxSetJointTargetVelocity(clientID, motorDireitoHandle,velocidade,vrep.simx_opmode_oneshot);
end

