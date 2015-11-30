%Script para cptura de teste com controle de velocidade.

position = rossubscriber('/RosAria/pose');
velocity = rossubscriber('/RosAria/cmd_vel');

t1 = 0;
t2 = 0;

for m =0:1000000000;
    m = m + 1;
end    

figure
for k = 1:3
    h(k) = subplot(3,1,k);
end
hold on
while 1
    vetor = receive(position);
    vetor_v = receive(velocity);
    
    x = vetor.Pose.Position.X;
    y = vetor.Pose.Position.Y;
    
    vl = vetor_v.Linear.X; 
    va = vetor_v.Angular.Z;
    
    hold on
    subplot(h(1))
    plot (x,y,'bo')
    title('Trajetória')
    hold off
    
    hold on
    subplot(h(2))
    plot (t1,vl,'r-o')
    title('Velocidade Linear')
    t1 = t1 + 1;
    hold off
    
    hold on
    subplot(h(3))
    plot (t2,va,'g-o')
    title('Velocidade Angular')
    t2 = t2 + 1;
    hold off
end
hold off
