function [Stack,item] = popStack(Stack,ii);
%Pop an item off the stack.
%    [Stack,item] = popStack(Stack,ii) returns item ii from the Stack and also
%    moves it to the top.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-18-2004      rme         Created and written.

% pop item from stack
item = Stack{ii};
% move ii to the top of the stack
Stack = {Stack{ii}, Stack{1:ii-1}, Stack{ii+1:end}};
