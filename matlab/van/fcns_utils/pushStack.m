function Stack = pushStack(Stack,item);
%Push an item onto the stack.
%    Stack = pushStack(Stack,item) pushes item onto the top of the Stack.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-18-2004      rme         Created and written.

% push item onto top of stack
Stack = {item, Stack{1:end-1}};
