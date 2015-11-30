position = rossubscriber('/RosAria/pose'); % lê posição no tópico escrito pelo nó .cpp
count = 0; % para criar um "time" no while e finalizar a execução.
figure
title('Trajetória do Robô - Controle Cinemático')
xlabel('Posição X (m)')
ylabel('Posição Y (m)')
hold on
while 1
    vetor = receive(position,0.5); % cria uma variável para receber os dados de position
    x = vetor.Pose.Position.X; % explora os pontos X de cada position
    y = vetor.Pose.Position.Y; % explora os pontos Y de cada position
    plot (x,y,'bo') % plota cada ponto com um 'o' na cor azul
    %if (x > -1.28)
        %if (y > 5.5)
            %break
    %    end
    %end
    %count=count+1;
    %if (count > 3000)
     %   break
    %end    
end
hold off