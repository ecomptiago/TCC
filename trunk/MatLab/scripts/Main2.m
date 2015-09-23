clear all;
clc;
warning('off','all')

disp('Inicio do programa');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,0,5);
desvioFIS = readfis('fuzzy_desvio_objeto.fis');

if (clientID>-1)            
			disp('Conectado a servidor API remoto do V-Rep');
            [MotorDireitoHandle, MotorEsquerdoHandle,PioneerLX_Handle]=init_vrep(clientID,vrep);   
            pingTime=0;
            LaserValues=0;
            AnguloObj=120;
            setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3);
            setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3);
            %FIM DA INICIALIZACAO DO SISTEMA E VARIAVEIS
            
            %INICIO DO CODIGO DE CONTROLE
            while(pingTime < 300)%caso delay entre matlab e vrep se torne muito grande, sai do loop.
                
                %LE VALORES DO LASER E ORIENTACAO DO ROBO
                [rtnHokuyo,LaserString]= vrep.simxGetStringSignal(clientID,'pointsPackedX',vrep.simx_opmode_oneshot_wait);
                if(rtnHokuyo == 0)
                    [LaserValues, LaserOrientation, LaserPosition] = getLaserValues(LaserString);
                    [setoresAngulares] = calcSetoresAngulares(LaserValues);
                end
                %FIM DA LEITURA DO LASER E ORIENTACAO DO ROBO
                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3);
                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3);
                %GIRA ROBO PARA ANGULO DESEJADO
                AnguloObj=120;
                while(giraRobo(clientID, vrep, MotorDireitoHandle, MotorEsquerdoHandle, AnguloObj, LaserOrientation) ~= 1)
                    [rtnHokuyo,LaserString]= vrep.simxGetStringSignal(clientID,'pointsPackedX',vrep.simx_opmode_oneshot_wait);
                    if(rtnHokuyo == 0)
                        [LaserValues, LaserOrientation, LaserPosition] = getLaserValues(LaserString);
                    else
                        printf('nao ha leitura disponivel, na tentativa de girar o robo');
                    end
                    pause(0.5);
                end
                %%
                %ANDA RETO ATÉ ALTURA DETERMINADA
                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3);
                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3);
                %while()
                %%
                %DESVIO DE OBSTACULOS
                %calcula se algum sesnor laser mediu um valor representando a eminencia de impacto
                if(ativaFuzzy(LaserValues))
                    while(ativaFuzzy(LaserValues))
                        
                        %atualiza o valor de LaserValues. Se nao ler nada, anda para frente e sai do loop.
                        [rtnHokuyo,LaserString]= vrep.simxGetStringSignal(clientID,'pointsPackedX',vrep.simx_opmode_oneshot_wait);
                        if(rtnHokuyo == 0)
                            [LaserValues, LaserOrientation, LaserPosition] = getLaserValues(LaserString);
                            [setoresAngulares] = calcSetoresAngulares(LaserValues);
                        else
                            disp('nao leu nada do pointsPackedX, ande reto');
                            setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3);
                            setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3);
                            break;
                        end
                        %EXECUTA O FUZZY ATE ALCANCAR PONTO OBJETIVO
                        %disp('FUZZY DEVE ENTRAR EM AÇÃO') ;
                        turnRate = evalfis([setoresAngulares(1), setoresAngulares(2), setoresAngulares(3), setoresAngulares(4), setoresAngulares(5), setoresAngulares(6)],desvioFIS);
                        if(turnRate < 0 && turnRate < 0.01)
                            turnRate
                            setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3+abs(turnRate/60));
                            setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3);
                        else
                            if(turnRate > 0 && turnRate > 0.01)
                                turnRate
                                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3);
                                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3+abs(turnRate/60));
                            else
                                disp('acabou fuzzy, ande reto');
                                setaVelocidadeEsquerda(clientID, vrep, MotorEsquerdoHandle, 0.3);
                                setaVelocidadeDireita(clientID, vrep, MotorDireitoHandle, 0.3);
                            end
                        end
                    end
                end
                %%
                %GIRA ROBO PARA ANGULO DESEJADO
                while(giraRobo(clientID, vrep, MotorDireitoHandle, MotorEsquerdoHandle, AnguloObj, LaserOrientation) ~= 1)
                    [rtnHokuyo,LaserString]= vrep.simxGetStringSignal(clientID,'pointsPackedX',vrep.simx_opmode_oneshot_wait);
                    if(rtnHokuyo == 0)
                        [LaserValues, LaserOrientation, LaserPosition] = getLaserValues(LaserString);
                    else
                        printf('nao ha leitura disponivel, na tentativa de girar o robo');
                    end
                    pause(0.5);
                end
            end
            %FIM DO CODIGO DE CONTROLE
end