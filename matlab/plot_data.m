clear all
clc
close all
cd '/home/chad/Documents/mujoco-3.1.6/myproject/QLSLIP_one_leg2'
system("./run");

% desktop
cd '/home/chad/Documents/mujoco-3.1.6/myproject/QLSLIP_one_leg2/data'
% labtop 
% cd '/home/chad/Documents/mujoco-2.2.1/myProject/Quad_Template/data'
filename = '/data.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);


%% 1 r tracking
figure(1);
subplot(2,1,1);
title("r - position tracking")
plot(Arr(500:m,1),Arr(500:m,2), 'r-' , Arr(500:m,1),Arr(500:m,3), 'b-' );
legend('Ref r','curr r')
ylabel("r position(m)")
xlabel("t(s)")
grid on 

% vel tracking r
subplot(2,1,2);
title("r - velocity tracking")
plot(Arr(500:m,1),Arr(500:m,4), 'r-' , Arr(500:m,1),Arr(500:m,5), 'b-' );
legend('Ref r vel','curr r vel')
ylabel("r vel(m/s)")
xlabel("t(s)")
grid on 

%% 2 velocity tracking r and th
figure(2);
subplot(2,1,1);
plot(Arr(500:m,1),Arr(500:m,6), 'r-' , Arr(500:m,1),Arr(500:m,7), 'b-' );
legend('Ref th','curr th')
ylabel("th position(rad)")
xlabel("t(s)")
grid on 

subplot(2,1,2);
plot(Arr(500:m,1),Arr(500:m,8), 'r-' , Arr(500:m,1),Arr(500:m,9), 'b-' );
legend('Ref th vel','curr th vel')
ylabel("th vel(m/s)")
xlabel("t(s)")
grid on 

%% 3. body vel tracking
figure(3);
subplot(3,1,1);
plot(Arr(500:m,1),Arr(500:m,10), 'r-' , Arr(500:m,1),Arr(500:m,11), 'b-' );
legend('Ref vel','Acutal vel')
ylabel("velocity(m/s)")
xlabel("t")
grid on 

% 
% figure(5);
% plot(Arr(500:m,1),Arr(500:m,6), 'r-' , Arr(500:m,1),Arr(500:m,7), 'b-' );
% legend('Ref vx','Acutal vx')
% ylabel("velocity")
% xlabel("t")
% grid on 
% 
% figure(6);
% plot(Arr(500:m,1),Arr(500:m,8), 'r-' , Arr(500:m,1),Arr(500:m,9), 'b-', Arr(500:m,1),Arr(500:m,10), 'g-', Arr(500:m,1),Arr(500:m,11), 'k-'   );
% legend('FL_td','FR_td','RL_td','RR_td')
% ylabel("th position")
% xlabel("t")
% grid on 