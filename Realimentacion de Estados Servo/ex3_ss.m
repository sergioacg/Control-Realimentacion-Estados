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
a=[2 3; 1 4];
b=[1 0;0 1];
c=[1 0;0 1];
d=0;

%Espacio de Estados del sistema
sys=ss(a,b,c,d);

%% Controlabilidad
Co = ctrb(sys)
rank(Co)



%% Sistema aumentado: Planta + Controlador
Aa = [a zeros(length(a));-c zeros(length(c))];
Ba = [b;zeros(length(c))];
Ea = [zeros(length(a));eye(length(a))];
Ca = [c zeros(length(c))];
sys_a =ss(Aa,[Ba Ea],Ca,0)

%% Realimentación de Estados
% Polos deseados con integrador
polos = roots(conv([1 1],conv([1 2],conv([1 3],[1 4]))));

%Realimentación de Estados ampliado
K = place(Aa,Ba,polos)

k1 = K(1:end-1)
k2 = -K(end)

%% Lazo cerrado
Af=Aa-Ba*K;

sys_f=ss(Af,Ea,Ca,0)

figure
step(tf(sys_f))

