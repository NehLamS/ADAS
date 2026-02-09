% Freinage reel
% Concevoir Kb
Kb = ?;

% Identifier la condition de freinage pour le EBD
% point A: adherence = -0.4 et 90% d'efficacité de freinage
FxrA = ux(2)*M*g*(l_f + hG*ux(2))/l;
FxfA = Kb*FxrA;
FxrAp = 0.9*FxrA;
FxfAp = Kb*FxrAp;

%Point B: adherence = -0.7
FxfB = ux(5)*M*g*(l_r - hG*ux(5))/l;
FxrB = ux(5)*M*g*(l_f + hG*ux(5))/l;

% Concevoir Kb'
Kbp = (FxfB - FxfAp)/(FxrB - FxrAp);

% Calculer l'efficacité du freinage pour le EBD
yIntercept = FxrAp - FxfAp/Kbp;

firstIncrement = FxfAp/30;
newFxf(1:31) = 0:firstIncrement:FxfAp;
newFxr(1:31) = newFxf(1:31)/Kb;

secondIncrement = (FxfB - FxfAp)/70;
newFxf(32:101) = (FxfAp+secondIncrement):secondIncrement:FxfB;
newFxr(32:101) = newFxf(32:101)/Kbp + yIntercept;

figure; hold on;
plot(pNFxf, pNFxr, 'o-');
plot(abs(newFxf(1:31)), abs(newFxr(1:31)), '--');
plot(abs(newFxf(32:101)), abs(newFxr(32:101)), '--');

% Comparer les valeurs de décélération
for i = 1:101
    newUx(i) = l/((M*g*l_r/newFxf(i)) - hG*(1 + newFxr(i)/newFxf(i)));
end

actualAcceleration = (newFxf + newFxr)/M;
idealAcceleration = newUx*g;

brakingEfficiency = actualAcceleration./idealAcceleration;

figure; hold on;
plot(abs(newUx), brakingEfficiency);
title('Braking Efficiency vs Longitudinal Slip');
xlabel('Longitudinal Slip [-]');
ylabel('Braking Efficiency [-]');
grid on;
xlim([0 1.2]);
ylim([0.4 1.2]);
