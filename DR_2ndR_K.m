%% load Data
clear; clc;
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; %왼쪽 뒷바퀴 휠속도
RR = exp1_014.Y(5).Data; %오른쪽 뒷바퀴 휠속도 % 두개를 이용해 Vk를 구한다.
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1); % gps데이터를 DR을 이용해 보정
Yaw_rate = exp1_014.Y(9).Data;
Time = exp1_014.X.Data;
%% calc yaw_drift
yaw_mean = mean(Yaw_rate(1:5944)); % 센서의 오차를 제거하기 위함.
%% initial Variable
x(1) = 0; % 초기 차량의 위치
y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % Km/h를 3.6으로 나눠서 m/s로 변환
wk = deg2rad(Yaw_rate-yaw_mean); % 센서의 오차를 제거하기 위해 yaw의 평균을 빼준다.
% time 변수에는 각 변수가 기록될 대의 시간이 기록.
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
%% 오일러
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; %왼쪽 뒷바퀴 휠속도
RR = exp1_014.Y(5).Data; %오른쪽 뒷바퀴 휠속도 % 두개를 이용해 Vk를 구한다.
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1); % gps데이터를 DR을 이용해 보정
Yaw_rate = exp1_014.Y(9).Data; % 
Time = exp1_014.X.Data;
yaw_mean = mean(Yaw_rate(1:5944)); % 센서의 오차를 제거하기 위함.
x(1) = 0; % 초기 차량의 위치
y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % Km/h를 3.6으로 나눠서 m/s로 변환
wk = deg2rad(Yaw_rate-yaw_mean); % 센서의 오차를 제거하기 위해 yaw의 평균을 빼준다.
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
RL = exp1_014.Y(4).Data; %왼쪽 뒷바퀴 휠속도
RR = exp1_014.Y(5).Data; %오른쪽 뒷바퀴 휠속도 % 두개를 이용해 Vk를 구한다.
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1); % gps데이터를 DR을 이용해 보정
Yaw_rate = exp1_014.Y(9).Data; % 
Time = exp1_014.X.Data;
yaw_mean = mean(Yaw_rate(1:5944)); % 센서의 오차를 제거하기 위함.
x(1) = 0; % 초기 차량의 위치
y(1) = 0;
th(1) = deg2rad(230);
vk = (RL+RR)/2/3.6; % Km/h를 3.6으로 나눠서 m/s로 변환
wk = deg2rad(Yaw_rate-yaw_mean); % 센서의 오차를 제거하기 위해 yaw의 평균을 빼준다.
for i = 1:178198
    th(i+1) = th(i) + wk(i)*(Time(i+1)-Time(i));
    x(i+1) = x(i)+(vk(i)/wk(i))*(sin(th(i+1))-sin(th(i)));
    y(i+1) = y(i)-(vk(i)/wk(i))*(cos(th(i+1))-cos(th(i)));
end
hold on;
plot(x,y);
legend('GPS','룽게-쿠타','오일러','exact integration');

%%
difference_x = find(xlunge ~= xouler);
difference_y = find(ylunge ~= youler);
fprintf("x좌표의 다른 지점 : %d\n",length(difference_x));
fprintf("y좌표의 다른 지점 : %d\n",length(difference_y));
