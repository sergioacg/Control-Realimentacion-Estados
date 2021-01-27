%% Realimentación de estados SIN la forma canónica controlable
%  by: Sergio Andres Castaño Giraldo
%  Rio de Janeiro - 2020
%  https://controlautomaticoeducacion.com/sistemas-dinamicos-lineales/control-por-realimentacion-de-estados/
% ______________________________________________________________________

clc
clear 
close all

%Sistema en lazo abierto NO esta en la forma FCC
A=[0 1 0 0;
   0 0 -1 0;
   0 0 0 1;
   0 0 5 0];
b=[0;1;0;-2];
c=[1 0 0 0];

%Espacio de Estados
sys=ss(A,b,c,0);

%% Controlabilidad
Co = ctrb(sys)
rank(Co)

%% Ecuación característica del Sistema
E_Ca=poly(eig(sys));

%% Ecuación deseada
Ps= [1 5 10.5 11 5];
Ed=roots(Ps)

%% Ganancia k_bar 
k_bar=Ps(2:end)-E_Ca(2:end);

%% Transformación de Similitud
Ci = toeplitz(E_Ca(1:end-1));
Ci = triu(Ci);
Q=(Co*Ci)^(-1)

%% Ganancia de Realimentación de Estados
k1 = k_bar*Q

%Puede ser calculado con comando place
k2 = place(A,b,Ed)

%% Lazo cerrado
Af=A-b*k1;

%% Calculando los autovalores del sistema en lazo cerrado
eig(Af)

%Condicion inicial
x0=[1 1 1 1];

%Sistema con realimentación en espacio de estados
slc=ss(Af,b,c,0);

%Respuesta del sistema
figure
subplot(211)
initial(sys,x0)
title('Lazo Abierto (CI)');
subplot(212)
initial(slc,x0)
title('Lazo Cerrado (CI)')

figure
subplot(211)
step(sys)
title('Lazo Abierto (Step)');
subplot(212)
step(slc)
title('Lazo Cerrado (Step)')
