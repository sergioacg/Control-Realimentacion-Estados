%% Realimentación de estados usando la ecuación de Lyapunov
%  by: Sergio Andres Castaño Giraldo
%  Rio de Janeiro - 2020
%  https://controlautomaticoeducacion.com/sistemas-dinamicos-lineales/control-por-realimentacion-de-estados/
% ______________________________________________________________________

clc
clear 
close all

% Sistema
A=[0 0; 1 0];
b=[1 0;0 1];
c=[1 0;0 1];

%Espacio de Estados
sys=ss(A,b,c,0);

%% Controlabilidad
Co = ctrb(sys)
rank(Co)

%% Ecuación característica del Sistema
E_Ca=poly(eig(sys));

%% Ecuación deseada
Ps= [1 7 3];
Ed=roots(Ps)

%% Ganancia k
k=Ps(2:end)-E_Ca(2:end)

%% Matriz F
F = [0,1;-3,-7];

%% Ganancia k Cualquiera arbitraria
k_bar=[0 1;1 0];

%% Matriz T
% beq = [0;1;0;0];
% Aeq = [0 3 1 0;
%       -1 7 0 1;
%       1 0 0 3;
%       0 1 -1 7];
% t=Aeq\beq;
% T=[t(1) t(2); t(3) t(4)];

%% Ganancia de la Realimentación de Estados
% k1 = k_bar/T %K_bar*inv(T)

%% Usando comando de Matlab lyap
% X = lyap(A,B,C) solves the Sylvester equation
% AX+XB+C=0
% The matrices A, B, and C must have compatible dimensions but need not be square.
% (Descomentar las dos instrucciones a continuación:)

        T=lyap(A,-F,-b*k_bar);
        k1 = k_bar/T

%% Lazo cerrado
Af=A-b*k1;

%% Calculando los autovalores del sistema en lazo cerrado
eig(Af)

%Condicion inicial
x0=[1 1];

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
