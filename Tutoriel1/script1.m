% script1.m
clear all; close all; clc

% Paramètre du véhicule
lf = 1; rf = 2; lr = 3; rr = 4;  
vehicleTTS  

% Braquage du véhicule
driver.mode = 'step';	           
driver.delta0 =	0;		          
driver.deltaf =	2*pi/180;         
driver.steertime = 0;	

% Paramètres de simulation
simulation.speed = 20;	         
simulation.vmodel = 'bike';       
simulation.tmodel = 'linear';     
simulation.g = 9.81;
t0 = 0;              %temps de début de simulation
tf = 1;              %temps de fin de simulation 
tstep = 0.001;       %pas de temsp d'intégration
t = t0:tstep:tf;     %définir le temps de simulation
x = zeros((tf - t0) / tstep + 1,2); %initialiser le vecteur d'état

% Intégrer les dérivé dxdt pour calculer les états x
for i = 1:(size(t,2) - 1)
  delta = steering(simulation, driver, x(i,:), t(i));
  alpha = slips(simulation, vehicle, x(i,:), delta);
  Fy    = tireforces(simulation, vehicle, alpha);
  dxdt  = derivs(simulation,vehicle,driver,x(i,:),Fy);
  x(i+1,:) = x(i,:) + tstep*dxdt;
end

% Tracer les résultats de simulation
figure; sgtitle('Linear Model, \delta = 2^{\circ}, v_x = 20 m/s')
subplot(2,1,1), plot(t,x(:,1),t,x(end,1)*ones(1, length(t)),'r--');
xlabel('Time(s)'); ylabel('v_y (m/s)'); legend('Actual','v_{y,ss}');
subplot(2,1,2), plot(t,x(:,2),t,x(end,2)*ones(1, length(t)),'r--');
xlabel('Time(s)'); ylabel('d\psi (rad/s)'); legend('Actual','d\psi_{ss}');  
