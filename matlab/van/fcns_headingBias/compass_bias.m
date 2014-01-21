function P = compass_bias(hdg_nav,hdg_gs)
% CBIAS = COMPASS_BIAS(HDG_NAV, HDG_GS) correction of bias based on
% the consistent reconstruction
% attempts to parametrize the correction hdg_gs - hdg_nav with a
% simple cyclic function

% 20040203 OP Created

% a more complete solution would also use rates and consider a
% fourier transform type of solution

nfreq = 5;
nsamps = length(hdg_nav);

% can be solved by LLS
% measurement matrix
M = zeros(nsamps,2*nfreq+1);
for k = 1:nfreq
  M(:,2*k-1:2*k) = [cos(k*hdg_nav) sin(k*hdg_nav)];
end
M(:,end) = ones(nsamps,1); % add a bias term

% solve for parameters
P = M\(hdg_gs-hdg_nav);


% corrected hdg
hdg_c = M*P + hdg_nav;

figure(3);
plot(hdg_nav,hdg_gs,'b.',hdg_nav,hdg_c,'g.');
grid on;

figure(4);
RTD = 180/pi;
plot(hdg_nav*RTD,(hdg_gs-hdg_nav)*RTD,'b.',hdg_nav*RTD,(hdg_c-hdg_nav)*RTD,'g.');
xlabel('hdg nav [deg]');
ylabel('hdg im - hdg nav [deg]');
grid on;


