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