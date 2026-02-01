% script3

%% Question 7
clear all; close all; clc
% Paramètre du véhicule
lf = 1; rf = 2; lr = 3; rr = 4;
vehicleTTS;

% Braquage du véhicule
driver.mode = 'step';
driver.delta0 = 0;
driver.deltaf =	1*pi/180;
driver.steertime = 0;

% Paramètres de simulation
simulation.speed = 20;
simulation.vmodel = 'bike';
simulation.g = 9.81;
t0 = 0; tf = 1; tstep = 0.01;
t = t0:tstep:tf;
xL = zeros((tf - t0) / tstep + 1,2);   %initialiser le vecteur d'état pour le modèle 2-roues
xNL = zeros((tf - t0) / tstep + 1,2);  %initialiser le vecteur d'état pour le modèle 4-roues

% Charge sur chaque pneumatique
Fz(lf) = ?;
Fz(rf) = ?;
Fz(lr) = ?;
Fz(rr) = ?;

% Euler integration
for i = 1:(size(t,2) - 1)
  simulation.tmodel = 'linear';
  deltaL = steering(simulation, driver, xL(i,:), t(i));
  alphaL = slips(simulation, vehicle, xL(i,:), deltaL);
  FyL = tireforces(simulation, vehicle, alphaL);
  dxdtL = derivs(simulation, vehicle, driver, xL(i,:), FyL);
  xL(i+1,:) = xL(i,:) + tstep*dxdtL;
    
  simulation.tmodel = 'fiala';
  deltaNL = steering(simulation, driver, xNL(i,:), t(i));
  alphaNL = slips(simulation, vehicle, xNL(i,:), deltaNL);
  FyNL = tireforces(simulation, vehicle, alphaNL, Fz);
  dxdtNL = derivs(simulation, vehicle, driver, xNL(i,:), FyNL);
  xNL(i+1,:) = xNL(i,:) + tstep*dxdtNL;
end
step_angle = driver.deltaf * 180/pi;
% Plot the yaw rate response
figure; plot(t, xL(:,2), 'k:', t, xNL(:,2), 'k.');
title(['Yaw rate response at ', num2str(step_angle), ' deg']);
xlabel('time (s)'); ylabel('d\psi (rad/s)');
legend('Linear', 'Nonlinear');

% Plot the yaw rate response
figure; plot(t, xL(:,1), 'k:', t, xNL(:,1), 'k.');
title(['Lateral speed response at ', num2str(step_angle), ' deg']);
xlabel('time (s)'); ylabel('vy (rad/s)');
legend('Linear', 'Nonlinear');

%% Question 8
% modification script_3

% Loop jusqu'à ce que l'erreur de la dynamique du lacet > 10%
while abs(0.1*(?)) >= abs(?)
  % intégrer ici le modèle linéaire et non-linéaire comme dans la question précédente
  % Incrementer l'angle de braquage
  driver.deltaf = driver.deltaf + 0.1*pi/180;
end

% Calculer l'accélération latérale
step_angle = driver.deltaf * 180/pi;
disp(['Lateral acceleration (linear) at ', num2str(step_angle),  ' degrees:']);
disp(?);

% Plot the yaw rate response
figure; plot(t, xL(:,2), 'k:', t, xNL(:,2), 'k.');
title(['Yaw rate response at ', num2str(step_angle), ' deg']);
xlabel('time (s)'); ylabel('yaw rate (rad/s)');
legend('Linear', 'Nonlinear');

% Plot the yaw rate response
figure; plot(t, xL(:,1), 'k:', t, xNL(:,1), 'k.');
title(['Lateral speed response at ', num2str(step_angle), ' deg']);
xlabel('time (s)'); ylabel('vy (rad/s)');
legend('Linear', 'Nonlinear');
