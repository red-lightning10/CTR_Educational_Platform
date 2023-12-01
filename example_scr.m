

E  = 1935*10^6* ones(1,2);     % Young's modulus of Nitinol [N/m^2]
OD = [3.3, 2.64] * 1e-3;  % tube outer diameters [m]
ID = [3.046, 2.386] * 1e-3;  % tube inner diameters [m]
k  = [17, 22];            % tube precurvatures [m^-1]
theta = [60 -30]*pi/180        % tube rotations [rad]

[chi, gamma] = linkcurvature(E, OD, ID, k, theta);
phi = atan2(gamma,chi)*180/pi


function [chi,gamma] = linkcurvature(E, OD, ID, k, theta)
    % *YOUR CODE HERE*
    numerator = [0 0];
    denominator = 0;
    for i = 1: length(OD)
        denominator = denominator + E(i)*0.25*pi*(OD(i)^4 - ID(i)^4);
        numerator(1) = numerator(1) + E(i)*0.25*pi*(OD(i)^4 - ID(i)^4)*k(i)*cos(theta(i));
        numerator(2) = numerator(2) + E(i)*0.25*pi*(OD(i)^4 - ID(i)^4)*k(i)*sin(theta(i));
    end
    chi = numerator(1)/denominator;
    gamma = numerator(2)/denominator;
end