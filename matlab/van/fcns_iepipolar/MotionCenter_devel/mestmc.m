function [residuals,osel,MAX_ANGLE] = mestmc(residuals,y);
%persistent MAX_ANGLE_PREV; 
%if isempty(MAX_ANGLE_PREV), MAX_ANGLE_PREV = -1; end
% simple M-estimator limits max cost
%MAX_ANGLE = 5*10^abs(y(3)); % degrees
%MAX_ANGLE = max(abs(median(residuals)),MAX_ANGLE_PREV);
%MAX_ANGLE_PREV = MAX_ANGLE;
MAX_ANGLE = median(abs(residuals));


% find outliers
osel = find(abs(residuals) >= MAX_ANGLE);

% limit the cost of outliers
residuals_orig  = residuals;
residuals(osel) = sign(residuals(osel))*MAX_ANGLE;

%disp('M-est residuals')
%[reshape(residuals_orig,1,[]); reshape(residuals,1,[])]
fprintf('MAX_ANGLE = %.2e\ty(3) = %.2e\n',MAX_ANGLE,y(3));

