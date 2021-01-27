%% Realimentación de estados en la forma canónica controlable
%  by: Sergio Andres Castaño Giraldo
%  Rio de Janeiro - 2020
%  https://controlautomaticoeducacion.com/sistemas-dinamicos-lineales/control-por-realimentacion-de-estados/
% ______________________________________________________________________

clc
clear 
close all

%Función de transferencia
G= tf(2,[1 3 1]);

%Sistema en lazo abierto
A=[-3 -1;1 0];
b=[1;0];
c=[0 2];

%Espacio de Estados
sys=ss(A,b,c,0);

%% Ubicación de los polos deseados
s=pole(G);
%Tiempo de establecimiento aproximado llegando al 2% del estado estable
ts=3.912/min(abs(s)); %(mas o menos 10 segundos)

%Definamos una respuesta subamortiguada para el sistema en lazo cerrado
zeta=0.707; %Factor de amortiguamiento (diseño)
tsd=ts*0.75; %Tiempo de establecimiento deseado (diseño)

wn=4/(tsd*zeta);
s1=-zeta*wn+wn*sqrt(zeta^2-1);
s2=-zeta*wn-wn*sqrt(zeta^2-1);

%Polos del sistema deseado
sf=[s1;s2];

%ecuación característica deseada
Pd=poly(sf)

% ley de control
k=[A(1,1)+Pd(2) A(1,2)+Pd(3)];

%Lazo cerrado
Af=A-b*k;
%El autovalor CERO va ser cancelado con el cero de la FT
eig(Af)

%Matriz de Observabildiad
Of=[c;c*Af]

%observable, 
rank(Of)

%Sistema con realimentación en espacio de estados
slc=ss(Af,[],c,0);

%Condicion inicial
x0=[1 1];

%Respuesta del sistema
initial(sys,x0)
hold on
initial(slc,x0)
legend('Lazo Abierto','Lazo Cerrado')
