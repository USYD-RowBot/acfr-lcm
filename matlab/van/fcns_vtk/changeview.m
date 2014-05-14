function [cpose,Xr] = changeview(cpose,Xw,x_rw);

cpose = cpose([4 5 6 1 2 3],:); % change from OP to RME

for ii=1:size(cpose,2);
  x_wci = cpose(:,ii);
  
  x_rci = head2tail(x_rw,x_wci);
  
  cpose(:,ii) = x_rci;
end;

cpose = cpose([4 5 6 1 2 3],:); % change from RME to OP

Hrw = [rotxyz(x_rw(4:6)), x_rw(1:3); 0 0 0 1];

Xr = Hrw * [Xw'; ones(1,size(Xw,1))];

Xr = Xr(1:3,:)';
