function bitprint(bits)
% BITPRINT  Prints binary representation of a variable.
%   BITPRINT(BITS) prints a binary representation of the value stored in
%   BITS in convention with big-endian bit ordering. i.e., a uint8 decimal
%   1 would be printed as 00000001
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09 Oct 2009     RME         Created and Written

s = char(zeros(128,1));
for n=1:inf
    try
        s(n) = sprintf('%d', bitget(bits,n));
    catch
        break;
    end
end
fprintf('%s\n', s(n-1:-1:1));

