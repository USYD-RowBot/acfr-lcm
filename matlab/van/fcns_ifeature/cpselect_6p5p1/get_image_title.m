function label = get_Imageitle(arg,arg_name)
%GET_IMAGE_TITLE gets title of image from its string name or variable name
%
%   LABEL = GET_IMAGE_TITLE(arg,arg_name) returns a string containing the
%   arg's name.
%
%   Class Support
%   -------------
%   ARG can be a string or a variable name. 
%
%   Examples
%   --------
%   INPUTNAME is a function that has to be called within the body of a
%   user-defined function. This line would be withing a parse_inputs
%   subfunction in a toolbox function.
%   inputImageName = get_Imageitle(varargin{1},inputname(1));

%   Copyright 1993-2003 The MathWorks, Inc.
%   $Revision: 1.2 $  $Date: 2003/01/17 16:28:31 $

if ischar(arg)
    [path, label] = fileparts(arg);
else 
    label = arg_name;
end


