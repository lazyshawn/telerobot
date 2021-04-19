clear
global Qi J_type BB SS SE S0
global cc c0 ce
global m0 inertia0 m inertia mass
global d_time
global Qe
global Gravity
global num_q Ez


%%% Unit vector in z-axis
Ez = [ 0 0 1 ]';

%%% Link Parameters Definition
BB = [ 0 1 ]; % connectivity of each link
SS = [ -1 1; % define parent link and child link
       0 -1 ];
   
S0 = [ 1 0 ]; % define the link connected to the base
SE = [ 0 1 ]; % define the link connected to the end-effector
J_type = [ 'R' 'R' ]; % define joint type 'R'-revolute joint, 'P'-prismatic joint
c0 = [ 0.5 0; % vector from the Base frame to the joint frame connected to the base satellite.
       0  0 ; % if the joint is not connected to the base, the values are zeros.
       0  0 ];% 
   
m0 = 1000; % mass of the base satellite

inertia0 = [ 166.67 0 0; % inertia of the base satellite with respect to the base frame
             0 166.67 0;
             0 0 166.67 ];
         
m = [ 70 30 ]; % mass of each link

mass = sum(m) + m0; % total mass of the system

Qi = [ 0 0;
       0 0;
       0 0 ]; % relationship 

inertia = [ 0.0625 0 0 0.0625 0 0; % inertia of the each link with respect to the link's frame
            0 4.1979 0 0 4.1979 0; % ex. 2 links, (3x3)x2 = 3x6 matrix
            0 0 4.1979 0 0 4.1979 ];

cc = zeros(3,2,2); % define the length of each link

cc(:,1,1) = [ -0.5 0 0 ]';
cc(:,1,2) = [ 0.5 0 0 ]';
cc(:,2,2) = [ -0.5 0 0 ]';
ce = [ 0 0.5;
       0 0;
       0 0 ];

Qe = [ 0 0;
       0 0;
       0 0 ];
   
   
% Initialize variables
q = [0 0]';
qd = zeros(2,1);
qdd = zeros(2,1);
v0 = [ 0 0 0 ]';%% Initial velocity of base
w0 = [ 0 0 0 ]';
vd0 = [ 0 0 0 ]';
wd0 = [ 0 0 0 ]';
vv = zeros(3,2);
ww = zeros(3,2);
vd = zeros(3,2);
wd = zeros(3,2);
R0 = [ 0 0 0 ]';
Q0 = [ 0 0 0 ]';
A0 = eye(3,3);
Te = zeros(3,2);
F0 = [ 0 0 0 ]';
Fe = zeros(3,2);
T0 = [ 0 0 0 ]';
tau = [ 0 0 ]';
Gravity = [ 0 0 0 ]';

%%% Prameters for controlling joints
desired_q = [-90*pi/180 -90*pi/180 ]';
Kp = 500;
Kd = 120;

%%% Number of links
num_q = length( q );
num_e = 1;
joints = j_num(num_e);

%%% File open
fidw = fopen('sample.dat','w');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Simulation Loop start
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Time for 1 step
d_time = 0.01;

for time = 0:d_time:8,
    
time

%%% Torque control
tau = Kp.*( desired_q - q ) - Kd.*qd;

%%% Forward dynamics
[ R0, A0, v0, w0, q, qd ] = f_dyn_rk2( R0, A0, v0, w0, q, qd, F0, T0, Fe, Te, tau );
%%% Forward kinematics

AA = calc_aa( A0, q );
RR = calc_pos( R0, A0, AA, q );
[ POS_e, ORI_e ] = f_kin_e( RR, AA, joints );

POS_e_Old = POS_e;
%%% Euler angle
Q0 = dc2rpy( A0' );

%%% Degree

 Q0 = dc2rpy(A0');
    Z0 = dc2rpy((ORI_e)');
    
    q_deg = q * 180/pi;
    qd_deg = qd * 180/pi;
    qdd_deg = qdd * 180/pi;
    Q0_deg = Q0 * 180/pi;
    Z0_deg = Z0 * 180/pi;
    fprintf(fidw,'%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n',time,Q0_deg,R0,POS_e,q_deg,Z0_deg,tau,qd_deg,qdd_deg);
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Simulation Loop end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Open file
fid = fopen('sample.dat','r');
%%% Storage data in matrix
tmp = fscanf(fid,'%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g',[21 inf]);


tmp = tmp';
%%% Close file
fclose(fid);
%%% Plot graphs
figure,
plot(tmp(:,1),tmp(:,2:4),'-');
title('Q0');
xlabel('Time [s]');
ylabel('Rotation of Base [deg]');
legend('Around x axis','Around y axis','Around z axis');

figure,
plot(tmp(:,1),tmp(:,5:7),'-');
title('R0');
xlabel('Time [s]');
ylabel('Position of Base [m]');
legend('x','y','z');

figure,
plot(tmp(:,1),tmp(:,8:10),'-');
title('POS_e');
xlabel('Time [s]');
ylabel('Position of End-effecter [m]');
legend('x','y','z');

figure,
plot(tmp(:,1),tmp(:,11:12),'-');
title('q');
xlabel('Time [s]');
ylabel('Angle of joints [deg]');
legend('Joint1','Joint2');


figure,
plot(tmp(:,1),tmp(:,13:15),'-');
title('Z0');
xlabel('Time [s]');
ylabel('Endpoint orientation [deg]');
legend('roll','pitch','yaw'); 

figure,
plot(tmp(:,1),tmp(:,16:17),'-');
title('torque');
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('joint1','joint2'); 

figure,
plot(tmp(:,1),tmp(:,18:19),'-');
title('angle velocity of joints');
xlabel('Time [s]');
ylabel('angle velocity of joints[m/s]');
legend('joint1','joint2'); 

figure,
plot(tmp(:,1),tmp(:,20:21),'-');
title('angle acceleration of joints');
xlabel('Time [s]');
ylabel('angle acceleration of joints[m/s]');
legend('joint1','joint2'); 


%%% EOF



