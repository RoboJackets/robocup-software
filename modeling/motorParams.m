%Params for 3 phase, Y-Connected, Trapezoidal Brushess DC Motor
%Model Built from this Doc: http://www.ijaestonline.com/admin/post_image/1387172494_State-Space_Based_Simulink_Modeling_of_BLDC_Motor_and_its_Speed_Control.pdf
%Motor Spec Sheet: http://www.maxonmotorusa.com/medias/sys_master/root/8813854326814/14-232-EN-Jun.pdf


p = 16; %%number of motor poles (NOT pole pairs)
r = 1.03/2; %% phase resistance of stator winding
l = (0.572/2)/1000; %% phase inductance of stator winding
j = .000001; %% combined rotor and wheel moment of inirtia (Kg*m2)
b = 8.921e-6; %%Dynamic Frictional Torque Constant (Nm/rad/sec)(will need to determine w/ tests)
ke = 1/(285*2*pi/60); %% Motor Back-emf Constant (V/rad/sec)
c0 = .001; %%Static Frictional Torque of BLDC Motor
kt = 33.5e-3;%% Torque Constant Nm/A
Vcc = 24; %%Maximum Voltage of system
encoderResolution = 12; %%Measured in bits
encoderSampleTime = .001; %%Measured in seconds
%% X' = AX + BU
%% Y = CX

%% Y = [ia, ib, ic, omega, theta]'
%% X = [ia, ib, omega, theta]'
%% U = [Vab - emfab, Vbc - emfbc, Te - Tl]'

A = [-r/l,0,0,0;
     0,-r/l,0,0;
     0,0,-b/j,0;
     0,0,1,0];
 
 B = [2/(3*l),1/(3*l),0; 
     -1/(3*l),1/(3*l),0; 
     0,0,1/j; 
     0,0,0];
 
 C = [1,0,0,0; 
     0,1,0,0; 
     -1,-1,0,0; 
     0,0,1,0; 
     0,0,0,1];
 
 D = [0,0,0; 
     0,0,0; 
     0,0,0; 
     0,0,0;
     0,0,0];
 
 