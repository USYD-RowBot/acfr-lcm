function [my_struct] = text2struct(filename, parse_format, varargin)
%TEXT2STRUCT Read formatted data from text file into a structure.
%   MY_STRUCT = TEXT2STRUCT(FILENAME,FORMAT,FIELDS) reads data in from
%   the file FILENAME into a structure MY_STRUCT containing fields
%   FIELDS.  FIELDS is a comma separated string argument list.
%
%   TEXT2STRUCT is built upon TEXTREAD and therefore can make use of all
%   of TEXTREAD's optional parameters available through the functional call:
%   MY_STRUCT = TEXT2STRUCT(FILENAME,FORMAT,OPTIONS,FIELDS) where
%   OPTIONS is a comma separated list of options following the format
%   specified in TEXTREAD.
%
%   Examples:
%    Suppose the text file foo.bar contains data in the following form:
%      06/20/2001 12:00:00 1 10.1 5 foo
%      06/20/2001 12:01:00 2 10.2 6 bar
%      06/20/2001 12:02:00 3 10.3 7 foobar
%
%    This could be read using the following command
%      my_struct = text2struct('foo.bar','%s %s %d %f %d %s', ...
%                              'datestr','timestr','x','y','z','string');
%
%    Read file and match Type literal while skipping the floats
%      my_struct = text2struct('foo.bar','%d/%d/%d %d:%d:%d %d %*f %d %s', ...
%                              'month','day','year','hour','minute','second', ...
%                              'x','z','string');
%
%    Read file skipping the first line using TEXTREAD's optional 'headerlines' parameter:
%      my_struct = text2struct('foo.bar','%s %s %d %f %d %s', ...
%                              'headerlines',1,'datestr','timestr','x','y','z','string');
%      
%   See also TEXTREAD.

%History
%Date          Who        Comment
%----------    ---        -----------------------------------
%2001/06/20    rme        Create

%determine the number of expected fields
num_args    = length(varargin);
num_scan    = length(findstr('%',parse_format));
num_skip    = length(findstr('%*',parse_format));
num_fields  = num_scan - num_skip;
num_options = num_args - num_fields;

%determine if optional TEXTREAD parameters were used
if num_fields == num_args
   fields = sprintf('%s ',varargin{:});
   cmd_str = sprintf('[%s] = textread(filename,''%s'');', fields, parse_format);
elseif num_fields < num_args
   fields = sprintf('%s ',varargin{(num_options+1):end});
   cmd_str = sprintf('[%s] = textread(filename,''%s'',varargin{1:num_options});', fields, parse_format);
else
   err_msg = sprintf('Parsing format does not match number of fields.');
   error(err_msg);
end
eval(cmd_str);

%convert TEXTREAD data to structure
cmd_str = 'my_struct = struct(';
for ii = (num_options+1):num_args
   buff = sprintf('iscell(%s)',varargin{ii});
   if eval(buff) %handle char arrays
      buff = sprintf('''%s'',str2mat(%s),', varargin{ii}, varargin{ii});
   else          %handle numerical arrays
      buff = sprintf('''%s'',%s,', varargin{ii}, varargin{ii});
   end
   cmd_str = strcat(cmd_str, buff);
end
%strip trailing comma and close command sring
cmd_str = strcat(cmd_str(1:end-1),');'); 
eval(cmd_str);