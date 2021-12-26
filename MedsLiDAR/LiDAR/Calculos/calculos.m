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
Caja = ["Blanca";"Cafe";"Negra";"Roja"];
LC = [CB,CC,CN,CR];
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

            [p1, p2] = polyfit(x,y,1);
            error_medicion = 0;
            for z = 1:length(x)
                error = abs(y(z)-(x(z)*p1(1)+p1(2)));
                error_medicion = error_medicion + error;
            end
            error_medicion = error_medicion/length(x);
            errores = [errores; error_medicion];
        end
        media = mean(errores);
        de = std(errores);
        medias = [medias; media];
        des = [des; de];
    end
    %scatter(metros,medias, 'DisplayName', strcat('Media Caja', Caja(j)));
    hold on
    scatter(metros, des, 'DisplayName', strcat('DE Caja', Caja(j)));
    title('Desviacion Estandar del Error');
    xlabel('Distancia de LiDar a la Caja [m]');
    ylabel('Error [m]')
    ylim([0,0.0015])
    %legend(strcat('Media', Caja(j)), strcat('DE', Caja(j)));
end
hold off
legend show
%% Solo un grafico

