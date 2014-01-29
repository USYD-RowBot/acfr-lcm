function [B] = restackmds(A, endtime)
     
  B = zeros(1,2*length(A));
  B(1:2:end) = A;
  B(2:2:end) = A;
     
  if(nargin == 2)
    B(1:end-1) = B(2:end);
    B(end)     = endtime;
  end 


