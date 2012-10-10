Metric = 'Det';
% magnitude
figure(2); clf;
plot(data.fni,data.(Metric).eif./data.(Metric).eif, 'k:', ...
     data.fni,data.(Metric).ci./data.(Metric).eif,  'b-', ...
     data.fni,data.(Metric).eif./data.(Metric).seif, 'r-.');
pbaspect([3,1,1]);
xlabel('Feature ID');
title('Plot Comparison of Covariance Approximations Magnitude');
legend('Ideal','Our Method','Markov Blanket');
switch lower(Metric);
case 'trace'; ylabel('tr( \Sigma ) / tr( \Sigma_{true} )');
case 'det'; ylabel('det( \Sigma ) / det( \Sigma_{true} )');
otherwise; error('unkown type');
end;

figure(3); clf;
plot(data.fni,log10(data.(Metric).eif./data.(Metric).eif), 'k:', ...
     data.fni,log10(data.(Metric).ci./data.(Metric).eif),  'b-', ...
     data.fni,log10(data.(Metric).seif./data.(Metric).eif), 'r-.');
legend('Ideal','Our Method','Markov Blanket');
pbaspect([3,1,1]);
xlabel('Feature ID');
title('Plot Comparison of Covariance Approximations Magnitude');
switch lower(Metric);
case 'trace'; ylabel('log of tr( \Sigma ) / tr( \Sigma_{true} )');
case 'det'; ylabel('log of det( \Sigma ) / det( \Sigma_{true} )');
otherwise; error('unkown type');
end;

figure(4); clf;
plot(data.fni,data.(Metric).eif./data.(Metric).eif, 'k:', ...
     data.fni,data.(Metric).ci./data.(Metric).eif,  'b-', ...
     data.fni,data.(Metric).seif./data.(Metric).eif, 'r-.');
set(gca,'Yscale','log');
legend('Ideal','Our Method','Markov Blanket');
pbaspect([3,1,1]);
xlabel('Feature ID');
title('Plot Comparison of Covariance Approximations Magnitude');
switch lower(Metric);
case 'trace'; xlabel('log of tr( \Sigma ) / tr( \Sigma_{true} )');
case 'det'; xlabel('log of det( \Sigma ) / det( \Sigma_{true} )');
otherwise; error('unkown type');
end;


figure(5); clf;
[freq,bin] = hist(log10([data.(Metric).ci./data.(Metric).eif,data.(Metric).seif./data.(Metric).eif]),200);
bar(bin,freq(:,1),'b');
hold on;
bar(bin,freq(:,2),'r');
hold off;
pbaspect([3,1,1]);
axis tight;
axis(axis);
legend('Our Method','Markov Blanket');
hold on;
plot([0;0],[0;10^6],'k:');
ytmp = ( max(freq(:,1))+max(freq(:,2)) )/2;
arrow([-0.2,ytmp],[0,ytmp]);
text(-0.21,ytmp+10,'Ideal','fontsize',12);
%line([0.2,0],[5,4]);
hold off;
title('Histogram Comparison of Covariance Approximations Magnitude');
switch lower(Metric);
case 'trace'; xlabel('tr( \Sigma ) / tr( \Sigma_{true} )');
case 'det'; xlabel('det( \Sigma ) / det( \Sigma_{true} )');
otherwise; error('unkown type');
end;


% shape
figure(10); clf;
plot(data.fni,data.normci,data.fni,data.normseif);
pbaspect(gca,[3,1,1]);
xlabel('Feature Number');
ylabel('')
title('Data Association: Comparison of Covariance Approximation Directionality');
legend('Our Method','Markov Blanket');

figure(12); clf;
[freq,bin] = hist([data.normci,data.normseif],100);
bar(bin,freq(:,1),'b');
hold on;
bar(bin,freq(:,2),'r');
hold off;
xlabel('');
title('Histogram Comparison of Covariance Approximations Directionality');
legend('Our Method','Markov Blanket');

figure(13); clf;
[freq,bin] = hist([data.normci,data.normseif],100);
bar(bin,freq,'stacked');
colormap([0 0 1; 1 0 0]);
xlabel('');
title('Histogram Comparison of Covariance Approximations Directionality');
legend('Our Method','Markov Blanket');
