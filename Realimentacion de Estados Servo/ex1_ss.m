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
a=1;
b=1;
c=1;
d=0;

%Espacio de Estados del sistema
sys=ss(a,b,c,d);

%Sistema aumentado: Planta + Controlador
Aa = [a 0;-c 0];
Ba = [b;0];
Ea = [0;1];
Ca = [1 0];
Da = 0;
De = 0;

sys_a =ss(Aa,[Ba Ea],Ca,[Da De])

% Polos deseados
polos = roots([1 3 2]);

%Realimentación de Estados
K = place(Aa,Ba,polos)

%% Lazo cerrado
Af=Aa-Ba*K;

sys_f=ss(Af,Ea,Ca,0)

figure
subplot(211)
step(sys)
title('Lazo Abierto (Step)');
subplot(212)
step(sys_f)
title('Lazo Cerrado (Step)')
