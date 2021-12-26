%% inicializar
clear all
clc
metros = [1.5, 1, 2.5, 2, 3.5, 3, 4];
CN = 0.33;
CR = 0.30;
CC = 0.20; 
CB = 0.19; % ???
%% extraccion de datos de la muestrar
Archivo = ["1_5";"1";"2_5";"2";"3_5";"3";"4"];
Caja = ["Cafe";"Negra";"Roja"];
LC = [CC,CN,CR];
for j = 1:length(Caja)
    medias = [];
    des = [];
    for k = 1:length(Archivo)
        load(strcat('C:\Users\juan_\Desktop\Seminario\Seminario\MedsLiDAR\LiDAR\Caja',Caja(j),'\', Archivo(k) , 'm\DistX.mat'));
        load(strcat('C:\Users\juan_\Desktop\Seminario\Seminario\MedsLiDAR\LiDAR\Caja',Caja(j),'\', Archivo(k) , 'm\DistY.mat'));

        largo_real = LC(j);
        errores = [];

        if(length(X(1,:)) >= length(Y(1,:)) )
            ld = length(Y(1,:));
        else 
            ld = length(X(1,:));
        end

        for muestra = 1:ld
            x1 = X(:,muestra);
            y1 = Y(:,muestra);

            %scatter(x1,y1)
            %title('Total de datos LiDar')
            %ylabel('Distancia m')
            %xlabel('Distancia m')

            index = find(abs(x1) < largo_real);
            x1 = x1(index(1):index(length(index)));
            y1 = y1(index(1):index(length(index)));
            x = x1;
            [y1,removed] = rmoutliers(y1,'median');
            for i = 1:length(removed)
                if(removed(i))
                    x(x==x1(i))=[];
                end
            end
            [x,removed] = rmoutliers(x,'median');
            y = y1;
            for i = 1:length(removed)
                if(removed(i))
                    y(y==y1(i))=[];
                end
            end

            %figure
            %scatter(x,y)
            %title('Datos procesados LiDar')
            %ylabel('Distancia m')
            %xlabel('Distancia m')
            %ylim([0,6])

            largo_medido = abs(x(1) - x(length(x)));
            error_medicion = abs(largo_real - largo_medido);

            errores = [errores; error_medicion];
        end
        media = mean(errores);
        de = std(errores);
        medias = [medias; media];
        des = [des; de];
    end
    figure
    scatter(metros,medias);
    hold on
    scatter(metros, des);
    title('Media y DE del Error para Caja '+ Caja(j));
    xlabel('Distancia de LiDar a la Caja [m]');
    ylabel('Error [m]')
    legend('Media', 'DE');
end
%% Grafico de los errores
scatter(metros,medias);
hold on
scatter(metros, des);
title('Media y DE del Error para Caja Negra, 271 muestras');
xlabel('Distancia de LiDar a la Caja [m]');
ylabel('Error [m]')
legend('Media', 'DE');