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
media = mean(errores)
de = std(errores)
medias = [medias; media];
des = [des; de]
%% Grafico de los errores
scatter(metros, abs(medias));
hold on
scatter(metros, des);
%% Calculo del error de la muestra
largo  = 0.3;%Roja %0.33;%Negra

largo_medido = data(1) - data(length(data));

error_medicion = largo - largo_medido
clear brushedData

%% Calculo de la media y desviacion del error

e=[-0.3379,-0.3358,-0.3358,-0.3392,-0.3417];

media = mean(e)
de = std(e)

%% Grafico de media y de para caja Roja

metros = [1.5, 1, 2.5, 2, 3.5, 3, 4];
medias = [-0.2009,-0.3121];
des = [0.0017,0.0044];

scatter(metros, abs(medias));
hold on
scatter(metros, des);

%% Grafico de media y de para caja negra

metros = [1.5, 1, 2.5, 2, 3.5, 3, 4];
medias = [-0.0512, -0.0469,	-0.0593, -0.0629,-0.0699,-0.0578,-0.0693];
des = [0.0075,0.0177,9.40e-4,0.0026,0.0016,0.0068,0.002];

scatter(metros, abs(medias));
hold on
scatter(metros, des);
