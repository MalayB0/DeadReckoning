%% load Data
clear; clc;
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; %���� �޹��� �ټӵ�
RR = exp1_014.Y(5).Data; %������ �޹��� �ټӵ� % �ΰ��� �̿��� Vk�� ���Ѵ�.
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1); % gps�����͸� DR�� �̿��� ����
Yaw_rate = exp1_014.Y(9).Data;
Time = exp1_014.X.Data;
%% calc yaw_drift
yaw_mean = mean(Yaw_rate(1:5944)); % ������ ������ �����ϱ� ����.
%% initial Variable
x(1) = 0; % �ʱ� ������ ��ġ
y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % Km/h�� 3.6���� ������ m/s�� ��ȯ
wk = deg2rad(Yaw_rate-yaw_mean); % ������ ������ �����ϱ� ���� yaw�� ����� ���ش�.
% time �������� �� ������ ��ϵ� ���� �ð��� ���.
% Ts = time(i)-time(i-1) = delta_t
%% Display GPS
figure();
plot(gpsX, gpsY);
%% Dead Reckoning Using 2nd Order Runge-Kutta
for i = 1:178198
    th(i+1) = th(i) + wk(i)*(Time(i+1)-Time(i));
    x(i+1) = x(i)+vk(i)*(Time(i+1)-Time(i))*cos(th(i)+wk(i)*(Time(i+1)-Time(i))/2);
    y(i+1) = y(i)+vk(i)*(Time(i+1)-Time(i))*sin(th(i)+wk(i)*(Time(i+1)-Time(i))/2);
end
xlunge = x;
ylunge = y;
hold on;
plot(x,y);
%% ���Ϸ�
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; %���� �޹��� �ټӵ�
RR = exp1_014.Y(5).Data; %������ �޹��� �ټӵ� % �ΰ��� �̿��� Vk�� ���Ѵ�.
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1); % gps�����͸� DR�� �̿��� ����
Yaw_rate = exp1_014.Y(9).Data; % 
Time = exp1_014.X.Data;
yaw_mean = mean(Yaw_rate(1:5944)); % ������ ������ �����ϱ� ����.
x(1) = 0; % �ʱ� ������ ��ġ
y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % Km/h�� 3.6���� ������ m/s�� ��ȯ
wk = deg2rad(Yaw_rate-yaw_mean); % ������ ������ �����ϱ� ���� yaw�� ����� ���ش�.
for i = 1:178198
    th(i+1) = th(i) + wk(i)*(Time(i+1)-Time(i));
    x(i+1) = x(i)+vk(i)*(Time(i+1)-Time(i))*cos(th(i));
    y(i+1) = y(i)+vk(i)*(Time(i+1)-Time(i))*sin(th(i));
end
xouler = x;
youler = y;
hold on;
plot(x,y);
%% exact integer
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; %���� �޹��� �ټӵ�
RR = exp1_014.Y(5).Data; %������ �޹��� �ټӵ� % �ΰ��� �̿��� Vk�� ���Ѵ�.
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1); % gps�����͸� DR�� �̿��� ����
Yaw_rate = exp1_014.Y(9).Data; % 
Time = exp1_014.X.Data;
yaw_mean = mean(Yaw_rate(1:5944)); % ������ ������ �����ϱ� ����.
x(1) = 0; % �ʱ� ������ ��ġ
y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % Km/h�� 3.6���� ������ m/s�� ��ȯ
wk = deg2rad(Yaw_rate-yaw_mean); % ������ ������ �����ϱ� ���� yaw�� ����� ���ش�.
for i = 1:178198
    th(i+1) = th(i) + wk(i)*(Time(i+1)-Time(i));
    x(i+1) = x(i)+(vk(i)/wk(i))*(sin(th(i+1))-sin(th(i)));
    y(i+1) = y(i)-(vk(i)/wk(i))*(cos(th(i+1))-cos(th(i)));
end
hold on;
plot(x,y);
legend('GPS','���-��Ÿ','���Ϸ�','exact integration');

%%
difference_x = find(xlunge ~= xouler);
difference_y = find(ylunge ~= youler);
fprintf("x��ǥ�� �ٸ� ���� : %d\n",length(difference_x));
fprintf("y��ǥ�� �ٸ� ���� : %d\n",length(difference_y));
