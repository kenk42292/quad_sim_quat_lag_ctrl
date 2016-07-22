
m=1.267;%0.468
g=9.81;
l=0.3048;%;0.225

Jxx=0.01403; %0.004856
Jyy=0.01463; %0.004856
Jzz=0.03871; %0.008801

k1 = 3*10^-6;%2.3469*10^-6;%2.98*10^-6;
k2 = 3*10^-6;%2.9258*10^-6;%2.98*10^-6;
k3 = 3*10^-6;%3.026*10^-6;%2.98*10^-6;
k4 = 3*10^-6;%2.887*10^-6;%2.98*10^-6;

b1 = 1.14*10^-7;%1.14*10^-7;
b2 = 1.14*10^-7;%1.14*10^-7;
b3 = 1.14*10^-7;%1.14*10^-7;
b4 = 1.14*10^-7;%1.14*10^-7;

d = 0.025;

params = [m;g;l;Jxx;Jyy;Jzz;k1;k2;k3;k4;b1;b2;b3;b4; d];

[q0_init, q1_init, q2_init, q3_init] = euler2quaternion(0, 0.6, 0.7);
q_init_norm = norm([q0_init, q1_init, q2_init, q3_init]);
q0_init = q0_init/q_init_norm;
q1_init = q1_init/q_init_norm;
q2_init = q2_init/q_init_norm;
q3_init = q3_init/q_init_norm;
initial_states=[0;0;0;0;0;0;q0_init;0;q1_init;0;q2_init;0;q3_init;0];


%%%SET PROPORTIONAL CONTROL PARAMS HERE
%0.4, 16 works well for othre paper's parameters
%1.2, 18 worked well for my mass and J's
pq = 1.2;%1.2;
pw = 18;%18;

%%%USER INPUTS
x_accel_ref = 0;
y_accel_ref = 0;
a_u = g;
wz_ref = 0;

%inputs = [x_accel; y_accel; z_accel; a_u];
inputs = [x_accel_ref; y_accel_ref; a_u];


sim('quad_control_sim_q.slx');

states_q = [x_out, x_dot_out, y_out, y_dot_out, z_out, z_dot_out, q0_out, q0_dot_out, q1_out, q1_dot_out, q2_out, q2_dot_out, q3_out, q3_dot_out];
ddot_q = [q0_ddot_out, q1_ddot_out, q2_ddot_out, q3_ddot_out];


draw3Dmotion(t, x_out, y_out, z_out, q0_out, q1_out, q2_out, q3_out);











