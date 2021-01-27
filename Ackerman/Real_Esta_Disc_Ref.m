% Control por VARIABLES DE ESTADO
% Realimentación de Estados + Referencia
% Tiempo Discreto

clc
clear all
close all

Ts = 1; %Periodo de Muestreo
B = [0 0 1 0.5];      %Numerador
A = [1 -1 0.01 0.12]; %Denominador
Gd = filt(B,A,Ts)

%% Espacio de estados
[Ad,Bd,Cd,Dd] = tf2ss(B,A)
% Ad = [0 1 0;
%       0 0 1;
%       -0.12 -0.01 1];
% Bd =[0;0;1];
% Cd=[0.5 1 0];
% Dd = 0;

%% Sistema aumentado: Planta + Controlador
Aa = [Ad Bd;zeros(1,length(Ad)) 0];
Ba = [zeros(length(Bd),1);1];
Ca = [Cd 0];
Da = [Dd 0];


sys_a =ss(Aa,Ba,Ca,0,Ts)

%% Controlabilidad
Co = [Ba Aa*Ba Aa^2*Ba Aa^3*Ba]
rank(Co)

%% Ecuación característica del Sistema
E_Ca=poly(eig(sys_a));

%% Realimentación de Estados
% Polos deseados sin integrador
Ed = [0.5+i*0.5;0.5-i*0.5;0.1;0.2];
% Ed = [0;0;0;0];
Ps = poly(Ed);

%% Ganancia k_bar 
k_bar=Ps(2:end)-E_Ca(2:end);

%% Transformación de Similitud
Ci = toeplitz(E_Ca(1:end-1));
Ci = triu(Ci);
Q=(Co*Ci)^(-1)

%% Ganancia de Realimentación de Estados
k = k_bar*Q
% k = place(Aa,Ba,Ed)
% k = acker(Aa,Ba,Ed)

%% Ganancias del controlador
Km = [Ad-eye(length(Ad)) Bd;
      Cd*Ad              Cd*Bd];
In = [zeros(1,length(k)-1) 1];
K = (k+In)/Km;

K1 = K(1:end-1)
K2 = K(end)

%% Lazo cerrado
% Af= [Ad                 Bd;
%      K1-K1*Ad-K2*Cd*Ad  1-K1*Bd-K2*Cd*Bd];
% Bf = [zeros(length(Bd),1);K2];
% Cf = Ca;
Af= [Ad-Bd*K1         Bd*K2;
     -Cd*Ad+Cd*Bd*K1  1-Cd*Bd*K2];
Bf = [zeros(length(Bd),1);1];
Cf = Ca;

sys_f=ss(Af,Bf,Cf,0,Ts)

figure
step(tf(sys_f))