%Sergio Castaño
%Realimentación de estados discreto
clc
clear 
close all

Ts=1;
%Sistema en lazo abierto
a=[0 1;-0.16 -1];
b=[0;1];
c=[1 0];
d=0;

%Espacio de Estados del sistema
sys=ss(a,b,c,d,Ts);


%% Realimentación de Estados
% Polos deseados sin integrador
polos = [0.5+i*0.5;0.5-i*0.5];

K1 = place(a,b,polos)
K2 = acker(a,b,polos)
%% Lazo cerrado
Af=a-b*K1;

slc=ss(Af,[],c,0,Ts);

%Condicion inicial
x0=[0 1];

%Respuesta del sistema
initial(slc,x0)