function [residuals,osel] = mestmc(residuals,y);

% simple M-estimator limits max cost
MAX_ANGLE = 5*10^abs(y(3)); % degrees

% find outliers
osel = find(abs(residuals) >= MAX_ANGLE);

% limit the cost of outliers
residuals_orig  = residuals;
residuals(osel) = sign(residuals(osel)).*MAX_ANGLE;

disp('M-est residuals')
[residuals_orig'; residuals']
fprintf('MAX_ANGLE = %.2f  y(3) = %.2e',MAX_ANGLE,y(3));

