function [ setoresAngulares ] = calcSetoresAngulares( LaserValues )
% transforma os 180 feixes da leitura do sensor laser, em 6 medias, cada
% uma representando uma secao de 30 graus (considerando que o sensor le 180
% valores, ou seja, 180 graus)
setoresAngulares=zeros(1,6);
totalMedidas = size(LaserValues, 2);
feixeLimiar = totalMedidas/6;
for i=1:totalMedidas
    if(i<=totalMedidas/6)
        setoresAngulares(1)= setoresAngulares(1)+ LaserValues(i);
    else
        if(i<=2*feixeLimiar)
            setoresAngulares(2)= setoresAngulares(2)+ LaserValues(i);
        else
            if(i<=3*feixeLimiar)
                setoresAngulares(3)= setoresAngulares(3)+ LaserValues(i);
            else
                if(i<=4*feixeLimiar)
                    setoresAngulares(4)= setoresAngulares(4)+ LaserValues(i);
                else
                    if(i<=5*(totalMedidas/6))
                        setoresAngulares(5)= setoresAngulares(5)+ LaserValues(i);
                    else
                        setoresAngulares(6)= setoresAngulares(6)+ LaserValues(i);
                    end
                end
            end
        end
    end
end
%calcula a media dos feixes de cada secao
for i=1:size(setoresAngulares,2)
   setoresAngulares(i) = setoresAngulares(i) / feixeLimiar; 
   if(setoresAngulares(i) > 3.5)
      setoresAngulares(i)=3.5; 
   end
end
end

