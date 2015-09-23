function [ Motor_DireitoHandle, Motor_EsquerdoHandle, PionnerLX_Handle] = init_vrep( clientID , vrep )
%INIT_VREP Initialize vrep variables, sensors and actuators.
% Busca os handles de todos os objetos do ambiente
            [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait);
            if (res==vrep.simx_return_ok)
				fprintf('Detectados %d objetos no ambiente\n',length(objs));
                
			else
				fprintf('A função API remota retornou o erro #%d\n',res);
            end
            
            [rtnPioneer,PionnerLX_Handle] = vrep.simxGetObjectHandle(clientID,'Pionner_LX',vrep.simx_opmode_oneshot_wait);
            if (rtnPioneer==vrep.simx_error_noerror)
                fprintf('Pionner_LX encontrado\n');
            else
                fprintf('Pionner_LX nao encontrado\n');
            end
            
            % Detecta os motores direito e esquerdo
            [rtnGps,Motor_DireitoHandle] = vrep.simxGetObjectHandle(clientID,'Motor_Direito',vrep.simx_opmode_oneshot_wait);
            if (rtnGps==vrep.simx_error_noerror)
                fprintf('Motor_Direito encontrado\n');
            else
                fprintf('Motor_Direito nao encontrado\n');
            end
            
            [rtnGps,Motor_EsquerdoHandle] = vrep.simxGetObjectHandle(clientID,'Motor_Esquerdo',vrep.simx_opmode_oneshot_wait);
            if (rtnGps==vrep.simx_error_noerror)
                fprintf('Motor_Esquerdo encontrado\n');
            else
                fprintf('Motor_Esquerdo nao encontrado\n');
            end
end

