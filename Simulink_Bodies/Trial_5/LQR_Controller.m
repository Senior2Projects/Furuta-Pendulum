%% ====== Physical Parameters (example from CAD + existing model) ======

L1 = 30/1000;       % Arm 1 length [m]
L2 = 100/1000;      % Arm 2 length [m]

% Masses (convert from grams to kg)

m_2 = 9.6/1000;              % Arm2
m_coupler = 8.48/1000;   % kg
m_encoder = 40.58/1000;  % kg
m_holder = 25.23/1000;   % kg

% COM locations from CAD [meters]
l1_cad = 0.035;                 % original Arm1 COM
l_coupler = 0.12669;            % coupler COM
l_encoder = 0.12656;            % encoder COM
l_holder = 0.10628;             % holder COM
l_rod_arm2 = 0.04344;           % middle of Arm2 rod portion

% Moments of inertia
I_motor = 2.05613759e-06;      % motor
J_arm1 = 6.1912e-10;           % original Arm1
J2_p = 2e-6;                   % original Arm2

% Damping
b_theta1 = 0.0077;
b_theta2 = 5e-5;

% Motor and gravity
Kt = 0.0199;    % motor constant
r = 29.5729;    % motor coil resistance
g = 9.81;

%% ====== Effective Arm1 Properties (including coupler, encoder, holder) ======
m_1_new = m_coupler + m_encoder + m_holder;
l1_new = (m_coupler*l_coupler + m_encoder*l_encoder + m_holder*l_holder) / m_1_new;
J0_new = (26^2*I_motor + J_arm1) ...         % motor + Arm1
         + m_coupler*l_coupler^2 + m_encoder*l_encoder^2 + m_holder*l_holder^2;

%% ====== Effective Arm2 Properties ======

l2_new = L2/2;  % Arm2 COM at middle
J2_new = J2_p + m_2*l2_new^2;

%% ====== Linearized Dynamics around Upright (theta2 = pi) ======
s = -1; % upright equilibrium
term_1 = J0_new * J2_new - (m_2^2)*(l1_new^2)*(l2_new^2);

% B matrix (input: motor voltage)
B31 = (J2_new / term_1)*(26*Kt / r);
B32 = s*m_2*l2_new*l1_new / term_1;
B41 = (s*m_2*l2_new*l1_new / term_1)*(26*Kt / r);
B42 = J0_new / term_1;

B = [0 0;
     0 0;
     B31 B32;
     B41 B42];

% A matrix
A32 = g*m_2^2*l2_new^2*l1_new / term_1;
A33 = -b_theta1*J2_new / term_1 - B31*(26^2*Kt^2 / r);
A34 = -s*b_theta2*m_2*l2_new*l1_new / term_1;
A42 = s*g*m_2*l2_new*J0_new / term_1;
A43 = -s*b_theta1*m_2*l2_new*l1_new / term_1 - B41*(26^2*Kt^2 / r);
A44 = -b_theta2*J0_new / term_1;

A = [0 0 1 0;
     0 0 0 1;
     0 A32 A33 A34;
     0 A42 A43 A44];

%% ====== LQR Design ======
Q = diag([50 1000 10 50]);  % tune as needed
R = 1;
K = lqr(A, B(:,1), Q, R);
disp('LQR gain K:');
disp(K);
save('K_matrix.mat','K'); % save for Simulink
