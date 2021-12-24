%% inicializar
clear all
clc
metros = [1.5, 1, 2.5, 2, 3.5, 3, 4];
medias = [];
des = [];

%% extraccion de datos de la muestrar
load('C:\Users\juan_\Desktop\Seminario\Seminario\MedsLiDAR\LiDAR\CajaRoja\4m\DistX.mat')
load('C:\Users\juan_\Desktop\Seminario\Seminario\MedsLiDAR\LiDAR\CajaRoja\4m\DistY.mat')

largo_real = 0.3;
errores = [];

if(length(X(1,:)) >= length(Y(1,:)) )
    ld = length(Y(1,:));
else 
    ld = length(X(1,:));
end

for muestra = 1:ld
    x1 = X(:,muestra);
    y1 = Y(:,muestra);

    index = find(abs(x1)<0.65);
    index = rmoutliers(index,'quartiles');
    x1 = x1(index(1):index(length(index)));
    x = x1;
    y1 = y1(index(1):index(length(index)));
    [y,removed] = rmoutliers(y1,'median');
    for i = 1:length(removed)
        if(removed(i))
            x(x==x1(i))=[];
        end
    end
    %scatter(x,y)
    largo_medido = abs(x(1) - x(length(x)));
    error_medicion = abs(largo_real - largo_medido);
    
    errores = [errores; error_medicion];
end
media = mean(errores);
de = std(errores);
medias = [medias; media];
des = [des; de]
%% Grafico de los errores
scatter(metros,medias);
hold on
scatter(metros, des);
title('Media y DE del Error para Caja _');
xlabel('Distancia de LiDar a la Caja [m]');
ylabel('Error [m]')
legend('Media', 'DE');