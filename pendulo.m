%% CARREGA DADOS

x_data = xlsread('Pendulo','Dados','D5:D204');
theta_data = xlsread('Pendulo','Dados','F5:F204');
tempo = xlsread('Pendulo','Dados','A5:A204');
dt = 0.01;

%% FILTRAGEM USANDO SGOLAY
% aplicando o filtro sobre os valores de leitura x e theta
% Grau de Polinomio 3
% com 7 pontos

x_amort = sgolayfilt (x_data,3,7);
theta_amort = sgolayfilt (theta_data,3,7);

%% FILTRAGEM USANDO FILTRO BUTTERWORTH

%dt = 0.01;          % periodo de amostragem
fs = 1/dt;          % frequência de amostragem
fc = 20;            % frequência de corte (Hz)
Wn = fc / (fs/2);   % normaliza em relação à Nyquist

[b, a] = butter(2, Wn);  % filtro de 2ª ordem

% filtfilt aplica o filtro para frente e para trás, removendo o atraso de fase
x_filt = filtfilt(b, a, x_data);         
theta_filt = filtfilt(b, a, theta_data);


%% estimar velocidade e aceleração linear a partir do deslocamento anterior

[lin,col] = size(x_filt);
x_vel = zeros(lin,col);
x_vel(2:end) = (x_filt(2:end) - x_filt(1:end-1))/dt;

x_acel = zeros(lin,col);
x_acel(2:end) = (x_vel(2:end) - x_vel(1:end-1))/dt;

theta_vel = zeros(lin,col);
theta_vel(2:end) = (theta_filt(2:end) - theta_filt(1:end-1))/dt;


%% Calculo do função y
g = 9.81;      %gravidade em m/s^2
Mp = 0.040;    %Massa do Pendulo em Kg
Mc = 0.200;    %Massa do Caarinno
L = 0.26;      %Comprimento do pendulo em m

y = (((1-((cos(theta_filt)).^2)).*x_acel*(Mc+Mp)))+ Mp*g*cos(theta_filt).*sin(theta_filt)- Mp*L*theta_vel.*theta_vel.*sin(theta_filt);

%% Calculo por Minimos quadrados (least squares)
% duty foi fixado em 70% (178/255)
% A formula final do sistema y = K * 0,7 + B * (-xvel)

Y = y';
X = [ones(1,lin)*0.7; -x_vel'];
W = Y*pinv(X);
K = W(1, 1);
Bx = W(1, 2);
Yh = W*X;


%% Plotagem de figuras

% Deslocamento Linear da dados colidos X amortecidos 
figure (1); 
plot(tempo,x_data,'r');
hold on
plot (tempo,x_amort,'b');
hold off

% Deslocamento Angular da dados colidos X amortecidos
figure(2);
plot(tempo,theta_data,'r');
hold on
plot (tempo,theta_amort,'b');
hold off

% Deslocamento angular em Graus
figure(3);
theta_graus = theta_amort * 180 / pi;
plot(tempo,theta_graus,'r');

%% Deslocamento Linear da dados colidos vs. dados amortecidos  vs. dados filtrados 
figure (4);
plot(tempo,x_data,'r');
hold on
plot (tempo,x_filt,'b');
plot (tempo,x_amort,'g');
hold off
xlabel('Tempo')
ylabel('Metros')

%% Deslocamento Angular da dados colidos vs. dados amortecidos vs. dados filtrados

figure(5);
plot(tempo,theta_data,'r');
hold on
plot (tempo,theta_filt,'b');
plot (tempo,theta_amort,'g');
hold off
xlabel('Tempo')
ylabel('Ângulo (radianos)')

%% Velocidade Linear
figure(6);
plot(tempo,x_vel,'r');

% Aceleração Linar
figure(7);
plot(tempo,x_acel,'r');

% Velocidade Angular
figure(8);
plot(tempo,theta_vel,'r');

% Comparação dos resultados da Função y em comparação com os dados da
% matriz W
figure(9);
plot (tempo, y,'r');
hold on
plot (tempo, Yh,'b');
hold off

%% CALCULO DA FUNÇÃO DE TRANSFERÊNCIA


%Define as matrizes do espaço de estados
A = [0 1  0 0;
    g*(Mp+Mc)/(L*Mc) 0 0 (Bx)/(L*Mc);
    0 0 0 1;
    (-Mp*g)/Mc 0 0 (Bx)/Mc];
B = [0; (-K)/(L*Mc); 0; K/Mc];
C = [1 0 0 0];
D = 0;

% Converte para função de transferência
[num, den] = ss2tf(A, B, C, D);

% Exibe a função de transferência
G = tf(num, den);

%K = W(1, 1);Bx = W(1, 2);

%% Sintonizando o controlador
%Teste 1: kd = 60; kp = 150; ki =60;
%Teste 2: kd = 60; kp = 150; ki =300;
%nume = [1.084 0.5054];
nume = [1.084 0.5054];
deno = [1 0.2332 -45.28 -12.32];
sys = tf(nume,deno);
kd = 60; kp = 150; ki =60;
numec = [kd kp ki];
denoc = [1 0];
Gc = tf(numec,denoc);
hold on

step(feedback(Gc*sys,1)) 
