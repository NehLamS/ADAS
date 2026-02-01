% script_5
clear all, close all, clc

%% question 14
% Paramètre du véhicule
lf = 1; rf = 2; lr = 3; rr = 4; 
vehicleTTS;

% Braquage du véhicule
driver.mode = 'path'; 
driver.Ke = 0.1; 
driver.Kpsi = 1.0; 

% Accélération du véhicule
driver.tmode = 'trail'; 
finTorques = [200 250 300 350];

% Paramètres de simulation
simulation.tmodel = 'fiala'; 
simulation.vmodel = 'fourwheel'; 
simulation.g = 9.81;

% Track parameters
load racingLineAndTrack; 
trackInfo.c = c; 
trackInfo.sLine = sLine; 
trackInfo.radius = 95;

% Charge sur chaque pneumatique
Fz(lf) = ?;
Fz(rf) = ?;
Fz(lr) = ?;
Fz(rr) = ?;

% Simuler la dynamique du véhicule en considérant les 4 couples
for k = 1:length(finTorques)    %pour chaque vitesse couple
  % définir le temps de simulation et initialiser le vecteur d'état  
  t0 = 0; tf = 30; tstep = 0.001; t = t0:tstep:tf;
  x = zeros(length(t),10);
  x_world = zeros(length(t),3); 
  vx_0 = 52;                  %vitesse longitudinale initiale
  x(1,3) = vx_0; 
  x(1,4:7) = vx_0/vehicle.Re; %vitesse des roues initiale
  fTorque = finTorques(k);
  % Euler integration
  i = 1;
  while x(i,8) < trackInfo.sLine(6) && abs(x(i,9)) < 50
    delta = steering(simulation, driver, x(i,:), t(i));
    Tq = torques(simulation, driver, x(i,:), trackInfo, fTorque);
    [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
    [Fy,Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo, Tq);
    x(i+1,:) = x(i,:) + tstep*dxdt;
    dxdt_world = derivs_world(x(i,:), x_world(i,:));
    x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
    i = i + 1;
  end
  t_sim(k) = t(i);

  % Supprimer les états inutilisés
  x(i+1:end,:) = []; 
  x_world(i+1:end,:) = [];
  N{k} = x_world(:,1); 
  E{k} = x_world(:,2);
  s{k} = x(:,8); 
  e{k} = x(:,9);
  vx{k} = x(:,3); 
  vy{k} = x(end,1);
  delta_psi{k} = x(:,10);
end

% Tracer les résultats de simulation
marker = {'k', 'k:', 'k--', 'k-.'};
figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, e{k}, marker{k});
end
hold off
title('Lateral error vs. distance along track (constant corner exit torque)');
xlabel('s (m)'); ylabel('e (m)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
       'Location', 'Best');

%% question 15
figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, vx{k}, marker{k});
end
hold off
title('vx vs. distance along track (constant corner exit torque)');
xlabel('s (m)'); ylabel('vx (m/s)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
       'Location', 'Best');
       
% Zoom
figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (constant corner exit torque) [zoomed]');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
      'Location', 'SouthWest');
ylim([-5 5]);

%% question 16
figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (constant corner exit torque)');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
       'Location', 'Best');

%% question 17
figure;
bar(finTorques, t_sim(1:length(finTorques)), 0.1, 'k');
title('Total time to complete track vs. final constant corner exit torque');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Torque (N*m)'); ylabel('Time (s)');

%% question 18
for i = 1:length(finTorques)
  V(i) = sqrt((vx{i}(end))^2 + vy{i}^2);
end
figure;
bar(finTorques, V, 0.1, 'k');
title('Exit speed vs. final constant corner exit torque');
ylim([min(V) - 1, max(V) + 1])
xlabel('Torque (N*m)'); ylabel('Speed (m/s)');

%% question 19
figure; 
hold on;
plot(insideLineE, insideLineN, 'k:', outsideLineE, outsideLineN, 'k:');
for k = 1:length(finTorques)
  plot(E{k}, N{k}, marker{k});
end
hold off
title('World coordinates: constant corner exit torque');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle paths', 'Location', 'Best');

%% question 21
driver.tmode = 'throttle';  % Couple moteur type rampe
finTorques = [200 600 1000];

for k = 1:length(finTorques)
  t0 = 0; tf = 30; tstep = 0.001; t = t0:tstep:tf;
  x = zeros(length(t),10);
  x_world = zeros(length(t),3); 
  vx_0 = 52;
  x(1,3) = vx_0; 
  x(1,4:7) = vx_0/vehicle.Re;
  fTorque = finTorques(k);
  i = 1;
  while x(i,8) < trackInfo.sLine(6) && abs(x(i,9)) < 50
    delta = steering(simulation, driver, x(i,:), t(i));
    Tq = torques(simulation, driver, x(i,:), trackInfo, fTorque);
    [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
    [Fy,Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo, Tq);
    x(i+1,:) = x(i,:) + tstep*dxdt;
    dxdt_world = derivs_world(x(i,:), x_world(i,:));
    x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
    i = i + 1;
  end
  t_sim(k) = t(i);
  % Supprimer les états inutilisés
  x(i+1:end,:) = []; 
  x_world(i+1:end,:) = [];
  N{k} = x_world(:,1); 
  E{k} = x_world(:,2);
  s{k} = x(:,8); 
  e{k} = x(:,9);
  vx{k} = x(:,3);
  vy{k} = x(end,1);
  delta_psi{k} = x(:,10);
end

figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, e{k}, marker{k});
end
hold off
title('Lateral error vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('e (m)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], 'Location', 'Best');

figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, vx{k}, marker{k});
end
hold off
title('vx vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('vx (m/s)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], 'Location', 'Best');

figure; hold on;
for k = 1:length(finTorques)
  plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
       [num2str(finTorques(3)), 'N*m'], 'Location', 'Best');

figure;
bar(finTorques, t_sim(1:length(finTorques)), 0.1, 'k');
title('Total time to complete track vs. final ramp corner exit torque');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Torque (N*m)'); ylabel('Time (s)');

for i = 1:length(finTorques)
  V(i) = sqrt((vx{i}(end))^2 + vy{i}^2);
end

figure;
bar(finTorques, V(1:length(finTorques)), 0.1, 'k');
title('Exit speed vs. final ramp corner exit torque');
ylim([min(V) - 1, max(V) + 1])
xlabel('Torque (N*m)'); ylabel('Speed (m/s)');

figure; hold on;
plot(insideLineE, insideLineN, 'k:', outsideLineE, outsideLineN, 'k:');
for k = 1:length(finTorques)
  plot(E{k}, N{k}, marker{k});
end
hold off
title('World coordinates: ramp corner exit torque');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle paths', 'Location', 'Best');

%% question 22
%Couple rampe en augmentant la vitesse d'entrée du virage
vxi = [51 52 53 54];
for k = 1:length(vxi)
  t0 = 0; tf = 30; tstep = 0.001; t = t0:tstep:tf;
  x = zeros(length(t),10);
  x_world = zeros(length(t),3); 
  vx_0 = vxi(k);
  x(1,3) = vx_0; 
  x(1,4:7) = vx_0/vehicle.Re;
  fTorque = 600;
  i = 1;
  while x(i,8) < trackInfo.sLine(6) && abs(x(i,9)) < 50
    delta = steering(simulation, driver, x(i,:), t(i));
    Tq = torques(simulation, driver, x(i,:), trackInfo, fTorque);
    [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
    [Fy,Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo, Tq);
    x(i+1,:) = x(i,:) + tstep*dxdt;
    dxdt_world = derivs_world(x(i,:), x_world(i,:));
    x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
    i = i + 1;
  end
  t_sim(k) = t(i);
  x(i+1:end,:) = []; 
  x_world(i+1:end,:) = [];
  N{k} = x_world(:,1); 
  E{k} = x_world(:,2);
  s{k} = x(:,8); 
  e{k} = x(:,9);
  vx{k} = x(:,3);  
  vy{k} = x(end,1);
  delta_psi{k} = x(:,10);
end

figure; hold on;
for k = 1:length(vxi)
  plot(s{k}, e{k}, marker{k});
end
hold off
title('Lateral error vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('e (m)');
legend([num2str(vxi(1)), 'm/s'], [num2str(vxi(2)), 'm/s'], ...
       [num2str(vxi(3)), 'm/s'], [num2str(vxi(4)), 'm/s'], ...
    'Location', 'Best');

figure; hold on;
for k = 1:length(vxi)
  plot(s{k}, vx{k}, marker{k});
end
hold off
title('vx vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('vx (m/s)');
legend([num2str(vxi(1)), 'm/s'], [num2str(vxi(2)), 'm/s'], ...
       [num2str(vxi(3)), 'm/s'], [num2str(vxi(4)), 'm/s'], ...
       'Location', 'Best');

figure; hold on;
for k = 1:length(vxi)
  plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(vxi(1)), 'm/s'], [num2str(vxi(2)), 'm/s'], ...
       [num2str(vxi(3)), 'm/s'], [num2str(vxi(4)), 'm/s'], ...
       'Location', 'Best');

figure;
bar(vxi, t_sim(1:length(vxi)), 0.1, 'k');
title('Total time to complete track vs. initial speed');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Speed (m/s)'); ylabel('Time (s)');

for i = 1:length(vxi)
  V(i) = sqrt((vx{i}(end))^2 + vy{i}^2);
end
figure;
bar(vxi, V(1:length(vxi)), 0.1, 'k');
title('Exit speed vs. initial speed for ramp corner exit torque');
ylim([min(V) - 1, max(V) + 1])
xlabel('Initial speed (m/s)'); ylabel('Final speed (m/s)');

figure; 
hold on;
plot(insideLineE, insideLineN, 'k:', outsideLineE, outsideLineN, 'k:');
for k = 1:length(vxi)
  plot(E{k}, N{k}, marker{k});
end
hold off
title('World coordinates: ramp corner exit torque w/ changing initial speed');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle paths', 'Location', 'Best');
