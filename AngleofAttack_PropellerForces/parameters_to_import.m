clear
clc;


% Basic Aircraft Parameters
params.mass = 13.5;
params.Jx = 0.8244;
params.Jy = 1.135;
params.Jz = 1.759;
params.Jxz = 0.1204;
params.S = 0.55;
params.b = 2.8956;
params.c = 0.18994;
params.S_prop = 0.2027;
params.rho = 1.2682;
params.k_motor = 80;
params.kT_p = 0;
params.kOmega = 0;
params.e = 0.9;

save('params.mat','-struct','params');

% Longitudinal Dynamics Coefficients
longitudinal_coeffs.CL0 = .28;
longitudinal_coeffs.CD0= 0.03;
longitudinal_coeffs.Cm0= -0.023338;
longitudinal_coeffs.CL_alpha= 3.45;
longitudinal_coeffs.CD_alpha= 0.30;
longitudinal_coeffs.Cm_alpha= -0.38;
longitudinal_coeffs.CL_q= 0;
longitudinal_coeffs.CD_q= 0;
longitudinal_coeffs.Cm_q= -3.6;
longitudinal_coeffs.CL_delta_e= -0.36;
longitudinal_coeffs.CD_delta_e= 0.0;
longitudinal_coeffs.Cm_delta_e= -0.5;
longitudinal_coeffs.C_prop= 1.0;
longitudinal_coeffs.M= 50;
longitudinal_coeffs.alpha_0= 0.4712;
longitudinal_coeffs.epsilon= 0.1592;
longitudinal_coeffs.CD_p= 0.0437;

save('longitudinal_coeffs.mat','-struct','longitudinal_coeffs');

% Lateral Dynamics Coefficients
lateral_coeffs.CY0= 0.;
lateral_coeffs.Cl0= 0.;
lateral_coeffs.Cn0= 0.;
lateral_coeffs.CY_beta= -0.98;
lateral_coeffs.Cl_beta= -0.12;
lateral_coeffs.Cn_beta= 0.25;
lateral_coeffs.CY_p= 0.;
lateral_coeffs.Cl_p= -0.26;
lateral_coeffs.Cn_p= 0.022;
lateral_coeffs.CY_r= 0.;
lateral_coeffs.Cl_r= 0.14;
lateral_coeffs.Cn_r= -0.35;
lateral_coeffs.CY_delta_a= 0.;
lateral_coeffs.Cl_delta_a= 0.08;
lateral_coeffs.Cn_delta_a= 0.06;
lateral_coeffs.CY_delta_r= -0.17;
lateral_coeffs.Cl_delta_r= 0.105;
lateral_coeffs.Cn_delta_r= -0.032;

save('lateral_coeffs.mat','-struct','lateral_coeffs');

