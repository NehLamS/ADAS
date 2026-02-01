% script4
%% questions 10 et 12
clear all, close all, clc
% Paramètre du véhicule
lf = 1; rf = 2; lr = 3; rr = 4;   
vehicleTTS;                       

% Braquage du véhicule
driver.mode = 'path';	          
driver.Ke = 0.1;                  
driver.Kpsi = 1.0; 

% Paramètres de simulation
simulation.tmodel = 'fiala';      
simulation.vmodel = 'fourwheel'; 
simulation.g = 9.81;

% Track parameters
load racingLineAndTrack;     %fichier contenant les coordionnées du circuit du test
trackInfo.c = c;             %courbure de la route
trackInfo.sLine = sLine;     %Coordonnée curviligne s
trackInfo.radius = 95;       %Rayon du virage

% Charge sur chaque pneumatique
Fz(lf) = ?;
Fz(rf) = ?;
Fz(lr) = ?;
Fz(rr) = ?;

% vitesse longitudinale maxi pour une accélération latérale de 0.8 m/s2
simulation.speed = sqrt(trackInfo.radius*0.8*simulation.g);
vx = simulation.speed;

% Augmente la vitesse par incréments de 0.5
speed = [vx, vx+0.5, vx+1.0, vx+1.5]; 

% Simuler la dynamique du véhicule en considérant les 4 vitesses
t_sim = zeros(length(speed),1);
marker = {'k', 'k:', 'k--', 'k-.'}; 

figure; hold on;
for k = 1:length(speed)         %pour chaque vitesse vx
  %définir le temps de simulation et initialiser le vecteur d'état    
  t0 = 0; tf = 30; tstep = 0.001;  t = t0:tstep:tf;
  x = zeros(length(t),10);  
  x(:,3) = speed(k);
  x(1,8) = 0;          
  x(1,9) = 1; 
  Fx = zeros(1,4);
    
  % Euler integration
  i = 1;
  while x(i,8) < trackInfo.sLine(6)
    delta = steering(simulation, driver, x(i,:), t(i));
    [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha, Fz);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo);
    dxdt(3) = 0;
    x(i+1,:) = x(i,:) + tstep*dxdt;
    i = i + 1;
  end

  t_sim(k) = t(i);
  x(i+1:end,:) = [];
  e = x(:,9);
  s = x(:,8);
  plot(s, e, marker{k});
end
hold off;
title('Lateral error vs. distance along track');
xlabel('Distance (m)'); ylabel('Lateral error (m)');
legend(['v_{x,max} = ', num2str(vx), 'm/s'], ...
       ['vx = ', num2str(vx+0.5), 'm/s'], ['vx = ', num2str(vx+1.0), 'm/s'], ... 
       ['vx = ', num2str(vx+1.5), 'm/s'], 'Location', 'North');

%% question 13
% à la suite du script_4
% Sortie du virage (e>3)
speed(end+1) = vx + 2.0;            
t0 = 0; tf = 30; tstep = 0.001;
t = t0:tstep:tf;
x = zeros(length(t),10);
x_world = zeros(length(t),3);     
x_world(1,2) = 1; 

x(:,3) = speed(end); 
x(1,8) = 0; 
x(1,9) = 1; 
Fx = zeros(1,4);

% Euler integration
i = 1;
while x(i,8) < trackInfo.sLine(6)
  delta = steering(simulation, driver, x(i,:), t(i));
  alpha = slips(simulation, vehicle, x(i,:), delta);
  Fy = tireforces(simulation, vehicle, alpha, Fz);
  dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo);
  dxdt(3) = 0;
  x(i+1,:) = x(i,:) + tstep*dxdt;
  dxdt_world = derivs_world(x(i,:), x_world(i,:));
  x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
  i = i + 1;
end

t_sim(length(speed)) = t(i);

% Supprimer les états inutilisés
x(i+1:end,:) = [];
x_world(i+1:end,:) = [];

% Tracer les résultats de simulation
e = x(:,9);
s = x(:,8);
figure; plot(s, e, 'k');
title('Lateral error vs. distance along track');
xlabel('Distance (m)'); ylabel('Lateral error (m)');
legend(['vx = ', num2str(speed(end)), 'm/s']);

figure; plot(insideLineE, insideLineN, 'k', ...
  outsideLineE, outsideLineN, 'k', x_world(:,2), x_world(:,1), 'k:');
title('Last simulation: world coordinates');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle path', 'Location', 'NorthWest');

figure;
bar(speed, t_sim, 0.1, 'k');
title('Total time to complete track vs. speed');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('vx (m/s)'); ylabel('Time (s)');
