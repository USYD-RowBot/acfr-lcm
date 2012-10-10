function hdg_c = compass_correction(hdg_nav,P)
% HDG_C = COMPASS_CORRECTION(HDG_NAV,P) corrects nav hdg using the
% parameter P (a set of coefficients of a fourrier series)

nparam = length(P);
nfreq = (nparam-1)/2;
nsamps = length(hdg_nav);
M = zeros(nsamps,nparam);
for k = 1:nfreq
  M(:,2*k-1:2*k) = [cos(k*hdg_nav) sin(k*hdg_nav)];
end
M(:,end) = ones(nsamps,1);

hdg_c = hdg_nav + M*P;
