function plot_nu_t(nu_t)
  
fignum = gcf;
fields = fieldnames(nu_t);
for xx=1:length(fields)
  % observation model function
  field = fields{xx};
  
  % parse to get sensor and observed state variables
  [junk,tok] = strtok(field,'_');
  [sensor,tok] = strtok(tok,'_');
  [variables,tok] = strtok(tok,'_');
  variables = cellstr(variables'); % convert string to cell array

  w = 30; % [s] sliding time window
  ww = find(nu_t.(field).t > nu_t.(field).t(end)-w);
  figure(fignum); clf;
  NN = size(nu_t.(field).nu,1);
  title(sprintf('Innovation: for %s\n',strrep(field,'_','\_')));
  for nn=1:NN
    % Normalized Innovation Squared Time-Average filter consistency test
    [epsilon,lb1,ub1] = nista_test(nu_t.(field).nu(nn,ww),nu_t.(field).S(nn,nn,ww));
    % White Sample Autocorrelation Time-Average filter consistency test
    %[rho,lb2,ub2] = wsata_test(nu_t.(field).nu(:,ww),1);
    rho = 0; lb2 = 0; ub2 = 0;
  
    subplot(NN,1,nn);

    hold on;
    % plot innovation
    plot(nu_t.(field).t,nu_t.(field).nu(nn,:),'-');
    plot(nu_t.(field).t,nu_t.(field).nu(nn,:),'+','MarkerSize',2);
  
    % plot innovation covariance std
    D  = squeeze(sqrt(nu_t.(field).S(nn,nn,:)));
    plot(nu_t.(field).t,D,'--');
    plot(nu_t.(field).t,-D,'--');
  
    % plot time window
    Ylim = get(gca,'Ylim');
    ct = nu_t.(field).t(end);
    line([ct, ct-w; ct, ct-w],[Ylim(1), Ylim(1); Ylim(2), Ylim(2)],'Color','r');
    
    hold off;
    grid on;
    title(sprintf('\nepsilon=%.2f [%.2f,%.2f]   \\rho(1)=%.2g [%.2g,%.2g]', ...
		  epsilon,lb1,ub1,rho(1),lb2,ub2));
    ylabel(variables{nn},'Rotation',0);
  
  end
  [epsilon,lb1,ub1] = nista_test(nu_t.(field).nu(:,ww),nu_t.(field).S(:,:,ww));
  xlabel(sprintf('Innovation: for %s\nepsilon=%.2f [%.2f,%.2f]   \\rho(1)=%.2g [%.2g,%.2g]', ...
		strrep(field,'_','\_'),epsilon,lb1,ub1,rho(1),lb2,ub2));
%  xlabel(sprintf('time [s]\nInnovation: for %s\n',strrep(field,'_','\_')));
  fignum = fignum+1;
end
