clear all; close all; clc

% freinage ideal
g = 9.81; theta = 0; l_f = 1.15; l = 2.3; l_r = l-l_f; hG = 0.5;
M = 1055;

% vecteur adherence dans l'intervalle [-0.2,-1.2] avec pas de 0.2
ux(1:6, 1) = -0.2:-0.2:-1.2;

% vecteur decelration dans l'intervalle [-0.2g,-1.2g] avec pas de 0.2g
ax = -0.2*g:-.2*g:-1.2*g; 

% Calculer les charges verticales
Fzf = M/l*(g*l_r*cos(theta) - g*hG*sin(theta) - hG*ax);
Fzr = M/l*(g*l_f*cos(theta) + g*hG*sin(theta) + hG*ax);

%Calculer la force de freinage totale pour tous les cas possibles
Fxf = ux*Fzf;
Fxr(1:6, 1:6, 1) = ux*Fzr;
for i = 1:6
   pNFxf(i) = -Fxf(i, i);  % on preleve des points pour les mettre sur la figure
   for j = 1:6
     % Forces de freinage pour ux1 constante
     Fxr(j, i, 2) = M*g*l_r*cos(theta)/hG - Fxf(j, i)*(l/ux(j)/hG +1);
     % Forces de freinage pour ux2 constante
     Fxr(j, i, 3) = ux(j)*M*g*l_f*cos(theta)/(l-ux(j)*hG) + Fxf(j, i)*(ux(j)*hG/(l-ux(j)*hG));
     % Forces de freinage pour ax constante
     Fxr(j, i, 4) = M*g*sin(theta) + M*ax(j) - Fxf(j, i);
   end
   pNFxr(i) = -Fxr(i, i);  % on preleve des points pour les mettre sur la figure
end

NFxf = -Fxf;
NFxr = -Fxr;

% Figure Fxr en fonction Fxf pour ux1 constante
figure; hold on;
for i = 1:6
  plot(NFxf(i, 1:6), NFxr(i, 1:6, 2), 'b--');
end
grid on;

% Figure Fxr en fonction Fxf pour ux2 constante
figure; hold on;
for i = 1:6
  plot(NFxf(1:6, i), NFxr(i, 1:6, 2), 'r--');
end
grid on;

% Figure Fxr en fonction Fxf pour ax constante
figure; hold on;
for i = 1:6
  plot(NFxf(i, 1:6), NFxr(i, 1:6, 4), 'k--');
end
grid on;

%courbe-I
figure; hold on;
plot(pNFxf, pNFxr, 'mo-');
for i = 1:6
  plot(NFxf(i, 1:6), NFxr(i, 1:6, 2), 'b--');
  plot(NFxf(1:6, i), NFxr(i, 1:6, 3), 'r--');
  plot(pNFxf, NFxr(1:6, i, 2), 'g--');
end
grid on;
