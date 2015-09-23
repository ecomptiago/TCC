function [ status ] = giraRobo( clientID, vrep, MotorDireitoHandle, MotorEsquerdoHandle, anguloObjetivo, LaserOrientation )
anguloRobo = 0;
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    status = 0;
    if((LaserOrientation(1)>0) && (LaserOrientation(3)>0))% apontando para norte
        anguloRobo=180+LaserOrientation(2);
        if(anguloRobo > 180)
            anguloRobo = LaserOrientation(2)-180;
        end
        %anguloRobo
    else %apontando para sul
        %anguloRobo = -LaserOrientation(2)
    end
    
    if(abs(anguloObjetivo - anguloRobo) <= 3)
        disp('chegou no angulo objetivo');
        setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0);
        setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0);
        status=1;
        return
    else
        %DEBUGdisp('virando');
        if(anguloObjetivo > anguloRobo)
            if(anguloObjetivo < 0)
                %DEBUGdisp('caso 1');
                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, -0.05);
                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.05);
            else
                %DEBUGdisp('caso 1');
                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, -0.05);
                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.05);
            end
        else
            if(anguloObjetivo < 0)
                %DEBUGdisp('caso 3');
                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.05);
                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, -0.05);
            else
                %DEBUGdisp('caso 4');
                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.05);
                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, -0.05);
            end
        end
        %DEBUGpause(0.3);
    end
    
end

