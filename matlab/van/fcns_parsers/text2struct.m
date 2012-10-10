function mystruct = text2struct(filename, parse_format, varargin);
%TEXT2STRUCT Read formatted data from text file into a structure.
%   MYSTRUCT = TEXT2STRUCT(FILENAME,FORMAT,FIELDS) reads data in from
%   the file FILENAME into a structure MY_STRUCT containing fields
%   FIELDS.  FIELDS is a comma separated string argument list.
%
%   TEXT2STRUCT is built upon TEXTREAD and therefore can make use of all
%   of TEXTREAD's optional parameters available through the functional call:
%   MYSTRUCT = TEXT2STRUCT(FILENAME,FORMAT,OPTIONS,FIELDS) where
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
%      mystruct = text2struct('foo.bar','%s %s %d %f %d %s', ...
%                             'datestr','timestr','x','y','z','string');
%
%    Read file and match Type literal while skipping the floats
%      mystruct = text2struct('foo.bar','%d/%d/%d %d:%d:%d %d %*f %d %s', ...
%                             'month','day','year','hour','minute','second', ...
%                             'x','z','string');
%
%    Read file skipping the first line using TEXTREAD's optional 'headerlines' parameter:
%      mystruct = text2struct('foo.bar','%s %s %d %f %d %s', ...
%                             'headerlines',1,'datestr','timestr','x','y','z','string');
%      
%   See also TEXTREAD.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-20-2001      rme         Created and written.
%    01-23-2006      rme         Updated code to be more readible and efficient.


% determine the number of expected fields
num_args    = length(varargin);
num_scan    = length(findstr('%',parse_format));
num_skip    = length(findstr('%*',parse_format));
num_fields  = num_scan - num_skip;
num_options = num_args - num_fields;

% determine if optional TEXTREAD parameters were used
if num_fields == num_args;
  textread_opts = {};         % no textread options
  fieldNames = {varargin{:}}; % user specfied structure fields
  %cmd_str = sprintf('[%s] = textread(filename,''%s'');', fields, parse_format);
elseif (num_fields < num_args);
  textread_opts = {varargin{1:num_options}};    % textread options
  fieldNames = {varargin{(num_options+1):end}}; % user specified structure fields
else;
  err_msg = sprintf('Parsing format does not match number of fields.');
  error(err_msg);
end;

% parse out the ASCII data using textread
[fieldData{1:num_fields}] = textread(filename,parse_format,textread_opts{:});
for ii=1:num_fields;
  if iscell(fieldData{ii});
    fieldData{ii} = char(fieldData{ii}); % convert string cell array to a char array
    %fieldData{ii} = {fieldData{ii}};    % keep string cell array as a string cell array
  end;
end;

% create a cell array of fieldname/fielddata pairs that can be used with STRUCT
[fieldPairs{1:2:2*num_fields}] = deal(fieldNames{:}); % field names
[fieldPairs{2:2:2*num_fields}] = deal(fieldData{:});  % field data

% convert TEXTREAD data to a structure
mystruct = struct(fieldPairs{:});
