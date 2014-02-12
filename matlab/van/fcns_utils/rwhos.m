function rwhos(varargin)
%RWHOS Recursively list current variables, long form.
%    RWHOS is a recursive form of WHOS.  It lists all the variables in the
%    current workspace, together with information about their size, bytes,
%    class, etc.
%     
%    RWHOS MAXLEVEL limits the recursive tree output to MAXLEVEL.
%    RWHOS ... VAR1 VAR2 restricts the display to the variables specified.
%  
%    The wildcard character '*' can be used to display variables that
%    match a pattern.  For instance, RWHOS A* finds all variables in the
%    current workspace that start with A.
%  
%    Use the functional form of RWHOS, such as RWHOS(V1,V2),
%    when the filename or variable names are stored in strings.
%
%    Example:
%    --------
%    rwhos 2 A*; % limit display to 2 levels and
%                % restrict search to variable in workspace that start
%                % with A.
%  
%    See also WHO, RWHOS.
%
%    Ryan M. Eustice 08-15-2003
%    Woods Hole Oceanographic Institution, MS 7
%    Woods Hole, MA 02543
%    508.289.3269   ryan@whoi.edu
% 
% History
% DATE          WHO                      WHAT
%----------    ------------------        ------------------------------
%2004-04-07    Ryan Eustice              Created & written.
%2004-04-08    rme                       Added issparse check.
%2004-12-17    rme                       Fixed a bug in argument parsing.

global RWHOS_MAX_TREE_DEPTH;

% check if tree depth for structures is specified
RWHOS_MAX_TREE_DEPTH = inf;
if nargin == 0;
  s_t = evalin('base','whos');
  varargin = {s_t.name};
elseif (nargin == 1) && (prod(size(varargin{1})) == 1) && ischar(varargin{1});
  RWHOS_MAX_TREE_DEPTH = str2num(varargin{1});
  s_t = evalin('base','whos');
  varargin = {s_t.name};  
elseif (nargin > 1) && (prod(size(varargin{1})) == 1) && ischar(varargin{1});
  RWHOS_MAX_TREE_DEPTH = str2num(varargin{1});
  varargin = {varargin{2:end}};
end;
if isstr(RWHOS_MAX_TREE_DEPTH)
  RWHOS_MAX_TREE_DEPTH = str2num(RWHOS_MAX_TREE_DEPTH);
end

fprintf('  Name                     Size                   Bytes  Class\n');
fprintf('\n');
recurse_whos(0,varargin{:})


%------------------------------------------------------------------
function recurse_whos(level,varargin);
global RWHOS_MAX_TREE_DEPTH;

% check if we are deeper than the maximum tree level
if level > RWHOS_MAX_TREE_DEPTH;
  return;
end;

for ii=1:length(varargin); % for each listed base workspace variable
  
  % get variable's information
  if isvarname(varargin{ii});
    cmd = sprintf('whos(''%s'')',varargin{ii});
    s_t = evalin('base',cmd);
  else;
    if ischar(varargin{ii});
      warning([varargin{ii},' is not a valid variable name']);
      continue;
    else;
      error('arguments must be strings');
    end;
  end;
  
  switch s_t.class;
   case 'struct'; % variable is a structure
    % print a summary and create a local copy of the structure
    print_whos(s_t);
    mystruct = evalin('base',s_t.name);
    
    % check for structure of arrays or an array of structures
    if prod(size(mystruct)) > 1;
      continue;
    end;
    
    fields = fieldnames(mystruct); % structure field names
    for jj=1:length(fields);
      % create a local copy of structure field jj
      cmd = sprintf('%s.(''%s'')',s_t.name,fields{jj});
      myfield = evalin('base',cmd);
      
      % assign a temporary variable in the base workspace
      % with the name mystruct_myfield
      varname = sprintf('%s_%s',s_t.name,fields{jj});
      assignin('base',varname,myfield);
      
      % recurse on the temporary variable
      recurse_whos(level+1,varname);
      
      % clean up temporary variable
      cmd = sprintf('clear(''%s'')',varname);
      evalin('base',cmd);
    end;
    
   case 'cell'; % variable is a cell
    print_whos(s_t);
    mycell = evalin('base',s_t.name);
   otherwise;   % variable is a standard class, e.g. double
    print_whos(s_t);
  end; % END switch s_t.class

end; % END for ii=1:length(varargin)


%-----------------------------------------------------------------
function print_whos(s_t);
% print a summary of variable memory usage
cmd = sprintf('size(%s)',s_t.name);
dim = evalin('base',cmd);
if length(s_t.name) < 25;
  fprintf('  %-25s',s_t.name);    % print variable name
else;
  fprintf('  %-50s',s_t.name);
end;
if size(dim > 1);               % print variable size
  size_str = sprintf('%dx',dim(1:end-1));
  size_str = strcat(size_str,sprintf('%d ',dim(end)));
else;
  size_str = sprintf('%d',dim);
end;
fprintf('%-16s',size_str);
fprintf('%+12s',num2str(s_t.bytes));  % print variable bytes
fprintf('  %-6s',s_t.class);          % print variable class
if prod(dim) > 1;
  fprintf(' array');
end;
cmd = sprintf('issparse(%s)',s_t.name);
if evalin('base',cmd);
  fprintf(' (sparse)');
end;
fprintf('\n');
