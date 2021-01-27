%% Realimentación de estados tipo SERVO
%  by: Sergio Andres Castaño Giraldo
%  Rio de Janeiro - 2020
%  https://controlautomaticoeducacion.com/sistemas-dinamicos-lineales
%  https://controlautomaticoeducacion.com/go/simulink-cupon/
% ______________________________________________________________________
clc
clear 
close all

%Sistema en lazo abierto
a=[0 1 0 0;
   0 0 -1 0;
   0 0 0 1;
   0 0 5 0];
b=[0;1;0;-2];
c=[1 0 0 0];
d=0;

%Espacio de Estados del sistema
sys=ss(a,b,c,d);

%% Controlabilidad
Co = ctrb(sys)
rank(Co)



%% Sistema aumentado: Planta + Controlador
Aa = [a zeros(length(a),1);-c 0];
Ba = [b;0];
Ea = [zeros(length(a),1);1];
Ca = [c 0];
sys_a =ss(Aa,[Ba Ea],Ca,0)

%% Realimentación de Estados
% Polos deseados con integrador
polos = roots(conv([1 5],[1 5 10.5 11 5]));

%Realimentación de Estados ampliado
K = place(Aa,Ba,polos)

k1 = K(1:end-1)
k2 = -K(end)

%% Lazo cerrado
Af=Aa-Ba*K;

sys_f=ss(Af,Ea,Ca,0)

figure
step(tf(sys_f))

