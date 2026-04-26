1. System Model
The system is inspired in bicopter drone models. This family of drones uses two propellers positionned at the extremities of the main body. The middle of the bar is connected to a rotation-axis, being the tilt angle the system output. Papers have presented control strategies for this system using classical controllers such as PID and state-feedback [ 1 ] [ 2 ]. This work provides a comparison of four techniques: PID, RST, State-feedback and Feedback Linearization plus State-feedback.

Fig 1. Seesaw system presented in [ 2 ].
The test bench developed for the laboratory uses drone motors as propellers and a potentiometer connected to the rotation center to do the angle measurement as presented in the Figure below.

Fig 2. Prototype for experimental tests in laboratory.
The Arduino UNO board is used as interface of the test bench. The algorithms and control laws are developed in the Simulink environnemnt and implemented by using the Connected I/O mode as suggested in [ 3 ]. In this mode the Arduino UNO is used only as interface of inputs and outputs of the test bench, all the signals are processed in the Simulink environnment. This feature is very interesting to bypass the limited power processor of a board like Arduino UNO, allowing the development and experimentation of control laws with high level of complexity. However, it generates some inconvenients, for example it is not possible to adjust the frequency of the PWM signals, then it is necessary an initial analysis of feasibility. 
The following pinout is used to connect the peripherals:
warning('off');

%%%% Pin numbers
pot_pin = 0 %A0 - Potentiometer
motor1 = 9 %D9 - Left Motor
motor2 = 10 %D10 - Right Motor 

%%%% Sampling control task
Ts = 5e-2

%%%% Sampling task potentiometer
Ts_a = Ts
In order to recalculate the angle, there are used the following constants:
%%%% Potentiometer constants
pot_offset = single(512)
pot_gain = single(pi/516)
The rotational speed is calculated by the classical speed definition. A low-pass filter can be used to smooth out the estimation.


Fig 3. Pre-processing of the potentiometer measurement.
In the same way, a post-processing of the control signal is used for the motor outputs. The driver uses two referenced pulses to control the motor:
1 ms for zero velocity
2 ms for maximal velocity
Note this is velocity, not torque, and the speed increases almost linearly between these values. The Connected I/O mode forces the output PWM for a frequency of 490.2 Hz. The Simulink block accepts an input between 0 and 255 for zero and maximum dutycyle of the PWM, respectively. The following constants are then defined to linearize an speed reference between 0 and 1 for a value between 0 and 255. To protect the components, it is assumed the maximal speed of the motor as a half of its total range which means 1.5 ms.
%%%% PWM constants
pwm_freq = single(490.2) 
min_dc = (1e-3/(1/pwm_freq))*255
max_dc = (1.5e-3/(1/pwm_freq))*255
pwm_gain = (max_dc-min_dc)

A initialization procedure is necessary to guarantee the driver behaves as expected, there are used these constants:
% Initialization variables
min_dc_init = (1e-3/(1/pwm_freq))*255
max_dc_init = (2e-3/(1/pwm_freq))*255
Note that, they linearize the control of motor speed. Nevertheless, the torque provided by the motors are the inputs of the model. Assuming some simplifications of the driver, as well that time constant of the motor dynamics is much faster than seesaw, the torque can be approximated as:

Where  is a constant linking the motor rotational speed and its torque. For simplicity, this constant is coupled to the gain that relates torque applied by the motor on the system. This means a feedfoward linearization can be applied in the system to bypass this non-linear behavior.

Fig 4. Post-processing of the control signals to generate the PWM wave.
Obs: It is needed attention in this initialization procedure. Sometimes, for unknown reasons until this moment, this process is not well executed in the Connected I/O mode, being necessary the use of the Run on Board mode to ensure the configuration of the driver.
Assuming the inputs as the torque provided by the motors () and the output as the angle related to the horizontal axis (), the system dynamics is described by the following non-linear equation:

where:
 is the system inertia
 is the dynamic friction coefficient
is the mass of the system
 is the gravity constant
 is the distance between the rotation axis and the gravity center of the system
 is the gain of the inputs
Nevertheless some coefficients are unknown, experimental tests were carried out to estimate these parameters. The results are illustrated in the following.
%% Model parameters
model.b = 0.25e-1;
model.M = 0.70;
model.L = 0.45;
model.g = 9.81;
model.x = 0.012;
model.J = (model.M*(2*model.L)^2)/12;
model.gain = 0.0016*(model.J)*0.9*10000;

%% Experimental results
load("DataExpe\Ensaio1.mat")
Ts_ensaio = round(mean(diff(Ensaio1.time)),2);
T_sim = Ensaio1.time(end);


2. Linearized system
From the equation presented in the Section 1, a linearized model can be extracted. Assuming:

where is the operating point of the system and  is the angle variation around this point. The simplest case is around the angle zero, which means:
  
The linear model, around zero, is then defined as:

%%%% Transfer function of the system
tf_model = tf(model.gain,[model.J model.b model.M*model.g*model.x]);
Another way to represent this model is by using the state-space model:
%%%% State space model of the system
ssmodel_A = [0 1; -(model.M*model.g*model.x)/model.J -model.b/model.J];
ssmodel_B = [0 model.gain/model.J; 0 -model.gain/model.J;]';
ssmodel_C = eye(2);
ssmodel_D = 0;
ssmodel = ss(ssmodel_A,ssmodel_B,ssmodel_C,ssmodel_D);

[result_lin,~,~,~] = lsim(ssmodel,Ensaio1.signals.values(:,[1 2]),Ensaio1.time);

2.1 Luenberger Observer
The rotational speed provided by the classical speed definition presents a high noise level which is stressfull for the control loops. The use of low-pass filters produces soften estimations, however it adds delay to the speed measurement. A simple Luenberger observer is then proposed to rebuild the speed waveform. Assuming the angle position as the only available sensor, the system observability is given by:

ssmodel_C = [1 0];
rank(obsv(ssmodel_A, ssmodel_C)) == size(ssmodel_A,1)
The poles of the observer are placed faster than the desired poles of the closed loop dynamics. A trade between velocity and noise amplification was made to choose its values. The place function is used compute the observer gain L.
p_observer = [-10 -9]; % s-plane
pd_observer = exp(p_observer*Ts); % z-plane
ssmodel_discrete = c2d(ssmodel,Ts,'zoh');
ssmodel_discrete_A = ssmodel_discrete.A;
ssmodel_discrete_B = ssmodel_discrete.B;

L_gain = place(ssmodel_discrete_A',ssmodel_C',pd_observer)';
