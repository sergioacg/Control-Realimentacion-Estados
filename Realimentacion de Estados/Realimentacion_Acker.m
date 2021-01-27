%% Realimentación de estados usando la ecuación de Ackerman
% Control por Variables de Estado
%  by: Sergio Andres Castaño Giraldo
%  Rio de Janeiro - 2020
%  https://controlautomaticoeducacion.com/sistemas-dinamicos-lineales/control-por-realimentacion-de-estados/
% ______________________________________________________________________

clc
clear 
close all

% Sistema
A = [0,1;1,0];
b = [1;0];
c=[1 0];


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
phi=0;
n=length(A);
for i=n:-1:0
    phi = phi + Ps(n-i+1)*A^i
end
k_bar=zeros(1,n);
k_bar(end)=1;

k1 = k_bar*inv(Co)*phi

%% Usando comando de Matlab acker
% (Descomentar las dos instrucciones a continuación:)

k2 = acker(A,b,Ed)

%% Lazo cerrado
Af=A-b*k1;

%% Calculando los autovalores del sistema en lazo cerrado
eig(Af);

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
