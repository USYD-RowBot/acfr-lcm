function Stack = initializeStack(stackSize);
%Create an empty stack.
%    Stack = initializeStack(stackSize) returns an empty stack.
%    To make the Stack persist through time, the stack variable
%    Stack should be declared persistent before invoking this function.
%
%    Example:
%    persistent Stack;
%    if isempty(Stack);
%      Stack = initializeStack(10);  % creat a 10 element memory stack
%    end;
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-18-2004      rme         Created and written.

Stack = cell(1,stackSize);
