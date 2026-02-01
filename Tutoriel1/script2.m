% script2.m
clear all; close all; clc
lf = 1; rf = 2; lr = 3; rr = 4;
vehicleTTS;
simulation.g = 9.81;

% Charge sur chaque pneumatique
Fz(lf) = ?;
Fz(rf) = ?;
Fz(lr) = ?;
Fz(rr) = ?;

% Focres latéral pneumatiques modèle linéaire et non-linéaire
alpha_step = 0.05;                 %pas d'angle 
alpha_final = 35;                  %angle de glissement latéral alpha final
angle = 0:alpha_step:alpha_final;  %définir un vecteur alpha
for i=1:size(angle,2)
  % Modèle pneumatique linéaire
  simulation.tmodel = 'linear'; 
  FyL(i,:) = tireforces(simulation,vehicle,angle(i)*pi/180.*ones(1,4));
  % Modèle pneumatique non-linéaire type Fiala
  simulation.tmodel = 'fiala';
  FyNL(i,:) = tireforces(simulation,vehicle,angle(i)*pi/180.*ones(1,4),Fz);   
end

% Tracer les résultats de simulation
figure; sgtitle('Total tire sideslip force')
subplot(2,1,1), plot(angle, FyL(:,1)+FyL(:,2), 'k:', angle, FyNL(:,1) + FyNL(:,2), 'k.-');
xlabel('\alpha (deg)'); ylabel('F_{yF} (N)');
legend('Linear', 'Nonlinear', 'Location', 'SouthWest');
subplot(2,1,2), plot(angle, FyL(:,3)+FyL(:,4), 'k:', angle, FyNL(:,3) + FyNL(:,4), 'k.-');
xlabel('\alpha (deg)'); ylabel('F_{yR} (N)');
legend('Linear', 'Nonlinear', 'Location', 'SouthWest');
