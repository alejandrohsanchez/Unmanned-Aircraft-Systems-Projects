clc
clear all
close all

load('params');
load('longitudinal_coeffs');
load('lateral_coeffs');

 % ---- START OF LOCAL VARIABLES ---- %
alpha = -30:1:50;
alpha = deg2rad(alpha);

Va = 5;
delta_e = 0;
q = 0;

beta = 10;
delta_r = 1;
delta_a = 1;
p = 1;
r = 1;

delta_t = 1;

phi = deg2rad(60);
theta = deg2rad(60);
%

% ---- END OF LOCAL VARIABLES ---- %
%
%
%
% ---- PART 1 ---- %

% Call to big function to calculate Fixed-Wing Forces

fly_WingsDynamics(Va, alpha, beta, p, q, r, phi, theta, delta_t, delta_a, delta_e, delta_r);




function [fx,fy,fz,l,m,n] = fly_WingsDynamics(Va, alpha, beta, p, q, r, phi, theta, delta_t, delta_a, delta_e, delta_r)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');

    % Writing the longitudinal forces as a function with the inputs as alpha,
    % Va, delta_e, q, and returning fx,fx, and m as outputs

    % This function calculates all the longitudinal forces (helper functions
    % included)
    Aerodynamic_Model = AM(alpha, Va, delta_e, q);

    plot(alpha,Aerodynamic_Model(1,:));
    hold on
    plot(alpha,Aerodynamic_Model(2,:));
    hold on
    plot(alpha,Aerodynamic_Model(3,:));
    hold off
    legend('fx','fz','m');
    xlabel('alpha (radians)')
    ylabel('unknown unit')
    title('Drag, Lift, and Pitching Moment vs Angle of Attack');
    subtitle('Test Case 3');
    annotation('textbox', [.15 .2 .4 .2], 'String', 'Va = 5, \alpha = -30 to 50, \beta = 10, p = 1, q = 0, r = 1, \phi = 60, \theta = 60, \delta_t = 1, \delta_a = 1, \delta_e = 0, \delta_r = 1')
    
    
    
    % Writing the lateral forces as a function with the inputs as beta, Va,
    % delta_r, delta_a, p, r, and returning fy, l, n, as outputs

    % This function calculates all the lateral forces (helper functions
    % included)

    Lateral_Aerodynamics = LA(beta, Va, delta_r, delta_a, p, r);
    fprintf("The following are the Lateral Aerodynamics values of the aircraft (fy, l, n, respectively)\n");
    disp(Lateral_Aerodynamics);
    
    
    
    
    
    
    % Writing the functions to calculate the propeller thrust and torque acting
    % upon the aircraft

    % This function calculates the propeller thrust (modeled as vector)
    Propeller_Thrust = PD(Va, delta_t);

    % This function calculates the propeller torque (modeled as vector)
    Propeller_Torque = PT(delta_t);

    fprintf("The following is a model of the propeller thrust acting upon the aircraft\n");
    disp(Propeller_Thrust);
    fprintf("The following is a model of the propeller torque acting upon the aircraft\n");
    disp(Propeller_Torque);

    fprintf("Merging all functions together including gravity to return fx, fy, fz, l, m, n\n");
    [fx,fy,fz,l,m,n] = COMBINE(Va, alpha, beta, p, q, r, phi, theta, delta_t, delta_a, delta_e, delta_r);
    A = [fx,fy,fz,l,m,n];
    disp(A);
end


function [fx,fy,fz,l,m,n] = COMBINE(Va, alpha, beta, p, q, r, phi, theta, delta_t, delta_a, delta_e, delta_r)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');
    g = 1;
    X = zeros(1,6);
    
    
    
    for i = 1:length(alpha)
        AR = (b .^2)/S;
        CDa = CD_p + (((CL0 + (CL_alpha * alpha(i))).^2)/(pi * e * AR));
        numerator = 1 + (e.^(-M * (alpha(i) - alpha_0))) + (e.^(M * (alpha(i) + alpha_0)));
        denominator = (1 + (e.^(-M * (alpha(i) - alpha_0)))) * (1 + (e.^(M * (alpha(i) + alpha_0))));
        sigma_alpha = numerator / denominator;
        CLa = ((1 - sigma_alpha) * (CL0 + (CL_alpha * alpha(i)))) + (sigma_alpha * (2 * sign(alpha(i)) * (sin(alpha(i)).^2) * cos(alpha(i))));
        const = 0.5 * rho * (Va.^2) * S;
        
        fx = const * (-CDa*cos(alpha(i))) + (CLa * sin(alpha(i))) + ((-CD_q * cos(alpha(i))) + (CL_q * sin(alpha(i))))*(c/(2*Va))*q + (-CD_delta_e * cos(alpha(i))) + (CL_delta_e * sin(alpha(i)))*delta_e;
        fx = (-mass * g * sin(theta)) + (const * fx);
        const2 = 0.5 * rho * S_prop * C_prop;
        const2 = const2 * (((k_motor * delta_t).^2) - (Va.^2));
        % fx
        fx = fx + const2;
        
        fy = const * (CY0 + (CY_beta * beta) + (CY_p * (b/(2*Va))*p) + (CY_r * (b/(2*Va))*r) + (CY_delta_a * delta_a) + (CY_delta_r * delta_r));
        % fy
        fy = (mass * g * cos(theta) * sin(phi)) + fy;
        
        fz = const * ((-CDa * sin(alpha(i))) - (CLa * cos(alpha(i))) + ((-CD_q * sin(alpha(i))) - (CL_q * cos(alpha(i))) * (c/(2*Va))*q) + (-CD_delta_e * sin(alpha(i))) - (CL_delta_e * cos(alpha(i))));
        % fz
        fz = fz + (mass * g * cos(theta) * cos(phi));
        
        l = b * (Cl0 + (Cl_beta * beta) + (Cl_p * (b/(2*Va))*p) + (Cl_r * (b/(2*Va))*r) + (Cl_delta_a * delta_a) + (Cl_delta_r * delta_r));
        % l
        l = (const * l) + (-kT_p * (kOmega * delta_t).^2);
        
        m = c * (Cm0 + (Cm_alpha * alpha(i)) + (Cm_q * (c/(2*Va))*q) + (Cm_delta_e * delta_e));
        % m
        m = const * m;
        
        n = b * (Cn0 + (Cn_beta * beta) + (Cn_p * (b/(2*Va))*p) + (Cn_r * (b/(2*Va))*r) + (Cn_delta_a * delta_a) + (Cn_delta_r * delta_r));
        % n
        n = const * n;
        
        X(1,1) = fx;
        X(1,2) = fy;
        X(1,3) = fz;
        X(1,4) = l;
        X(1,5) = m;
        X(1,6) = n;
    end
end
function mp = PT(delta_t)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');
    mp = [(-kT_p * (kOmega * delta_t).^2); 0; 0];
end

function Fp = PD(Va, delta_t)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');

    const = 0.5 * rho * S_prop * C_prop;
    Fp = [(((k_motor * delta_t).^2) - (Va.^2)); 0; 0];
end

function B = LA(beta, Va, delta_r, delta_a, p, r)
    B = [fy_lateral(beta, Va, delta_r, delta_a, p, r);
        l_lateral(beta, Va, delta_r, delta_a, p, r);
        n_lateral(beta, Va, delta_r, delta_a, p, r)];
end

function FY = fy_lateral(beta, Va, delta_r, delta_a, p, r)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');

    const = (0.5 * rho * (Va.^2) * S);

    FY = const * (CY0 + (CY_beta * beta) + (CY_p * (b/(2*Va))*p) + (CY_r * (b/(2*Va))*r) + (CY_delta_a * delta_a) + (CY_delta_r * delta_r));
end

function L = l_lateral(beta, Va, delta_r, delta_a, p, r)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');

    const = (0.5 * rho * (Va.^2) * S * b);

    L = const * (Cl0 + (Cl_beta * beta) + (Cl_p * (b/(2*Va))*p) + (Cl_r * (b/(2*Va))*r) + (Cl_delta_a * delta_a) + (Cl_delta_r * delta_r));
end

function N = n_lateral(beta, Va, delta_r, delta_a, p, r)
    load('params');
    load('longitudinal_coeffs');
    load('lateral_coeffs');

    const = (0.5 * rho * (Va.^2) * S * b);

    N = const * (Cn0 + (Cn_beta * beta) + (Cn_p * (b/(2*Va))*p) + (Cn_r * (b/(2*Va))*r) + (Cn_delta_a * delta_a) + (Cn_delta_r * delta_r));
end

function A = AM(alpha, Va, delta_e, q)
    A = zeros(2,length(alpha)-1);
    for i = 1:length(alpha)
        A(1,i) = F_drag(alpha(i), Va, delta_e, q);
        A(2,i) = F_lift(alpha(i), Va, delta_e, q);
        A(3,i) = pitch(alpha(i), Va, delta_e, q);
    end
end

function FD = F_drag(alpha, Va, delta_e, q)
    load('params');
    load('longitudinal_coeffs');

    AR = (b .^2)/S;
    CDa = CD_p + (((CL0 + (CL_alpha * alpha)).^2)/(pi * e * AR));

    % Blending Function %
    numerator = 1 + (e.^(-M * (alpha - alpha_0))) + (e.^(M * (alpha + alpha_0)));
    denominator = (1 + (e.^(-M * (alpha - alpha_0)))) * (1 + (e.^(M * (alpha + alpha_0))));
    sigma_alpha = numerator / denominator;
    CLa = ((1 - sigma_alpha) * (CL0 + (CL_alpha * alpha))) + (sigma_alpha * (2 * sign(alpha) * (sin(alpha).^2) * cos(alpha)));
    const = (0.5 * rho * (Va.^2) * S);

    FD = const * (((-CDa * cos(alpha)) + (CLa * sin(alpha))) + (((-CD_q * cos(alpha)) + (CL_q * sin(alpha))) * ((c/(2*Va))*q)) + (((-CD_delta_e * cos(alpha)) + (CL_delta_e * sin(alpha)))*delta_e));
end

function FL = F_lift(alpha, Va, delta_e, q)
    load('params');
    load('longitudinal_coeffs');

    AR = (b .^2)/S;
    CDa = CD_p + (((CL0 + (CL_alpha * alpha)).^2)/(pi * e * AR));

    % Blending Function %
    numerator = 1 + (e.^(-M * (alpha - alpha_0))) + (e.^(M * (alpha + alpha_0)));
    denominator = (1 + (e.^(-M * (alpha - alpha_0)))) * (1 + (e.^(M * (alpha + alpha_0))));
    sigma_alpha = numerator / denominator;
    CLa = ((1 - sigma_alpha) * (CL0 + (CL_alpha * alpha))) + (sigma_alpha * (2 * sign(alpha) * (sin(alpha).^2) * cos(alpha)));
    const = (0.5 * rho * (Va.^2) * S);

    FL = const * (((-CDa * sin(alpha)) - (CLa * cos(alpha))) + (((-CD_q * sin(alpha)) - (CL_q * cos(alpha))) * ((c/(2*Va))*q)) + (((-CD_delta_e * sin(alpha)) - (CL_delta_e * cos(alpha)))*delta_e));
end

function m = pitch(alpha, Va, delta_e, q)
    load('params');
    load('longitudinal_coeffs');

    const = (0.5 * rho * (Va.^2) * S * c);
    m = const * (Cm0 + (Cm_alpha * alpha) + (Cm_q * (c/(2*Va))*q) + (Cm_delta_e * q));

end
