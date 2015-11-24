position = rossubscriber('/turtle1/pose'); % lê posição no tópico escrito pelo nó .cpp
count = 0; % para criar um "time" no while e finalizar a execução.
figure
title('Trajetória do Robô - Controle Cinemático')
xlabel('Posição X (m)')
ylabel('Posição Y (m)')
hold on
while 1
    vetor = receive(position); % cria uma variável para receber os dados de position
    x = vetor.X; % explora os pontos X de cada position
    y = vetor.Y; % explora os pontos Y de cada position
    plot (x,y,'bo') % plota cada ponto com um 'o' na cor azul
    count=count+1;
    if (count > 2000)
        break
    end    
end
hold off