function varargout = trackview(varargin)
% TRACKVIEW M-file for trackview.fig
%      TRACKVIEW, by itself, creates a new TRACKVIEW or raises the existing
%      singleton*.
%
%      H = TRACKVIEW returns the handle to a new TRACKVIEW or the handle to
%      the existing singleton*.
%
%      TRACKVIEW('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACKVIEW.M with the given input arguments.
%
%      TRACKVIEW('Property','Value',...) creates a new TRACKVIEW or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trackview_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trackview_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trackview

% Last Modified by GUIDE v2.5 30-Sep-2003 21:01:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trackview_OpeningFcn, ...
                   'gui_OutputFcn',  @trackview_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
  gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
  [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
  gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before trackview is made visible.
function trackview_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trackview (see VARARGIN)

% Choose default command line output for trackview
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% rme
% add function to matlab path
[directory,filename] = fileparts(mfilename('fullpath'));
addpath(directory);

% rme
set(handles.figure1,'DoubleBuffer','On');
set(handles.figure1,'WindowButtonDownFcn',{@showfig,handles});
set(handles.figure1,'KeyPressFcn',{@advance_sequence,handles});
set(handles.figure1,'Resize','on');

% UIWAIT makes trackview wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = trackview_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function Inc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Inc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes during object creation, after setting all properties.
function Extension_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Extension (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes during object creation, after setting all properties.
function LeftNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LeftNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes during object creation, after setting all properties.
function RightNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RightNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function LeftNum_Callback(hObject, eventdata, handles)
% hObject    handle to LeftNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LeftNum as text
%        str2double(get(hObject,'String')) returns contents of LeftNum as a double
imgnum = str2double(get(handles.LeftNum,'String'));
update_left_images(imgnum, handles, 0);


function RightNum_Callback(hObject, eventdata, handles)
% hObject    handle to RightNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of RightNum as text
%        str2double(get(hObject,'String')) returns contents of RightNum as a double
increment = str2double(get(handles.Inc,'String'));
imgnum = str2double(get(handles.RightNum,'String'));
update_right_images(imgnum, handles, 0);


function Inc_Callback(hObject, eventdata, handles)
% hObject    handle to Inc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Inc as text
%        str2double(get(hObject,'String')) returns contents of Inc as a double
%increment = str2double(get(handles.Inc,'String'));
%imgnum = str2double(get(handles.LeftNum,'String'));
%update_left_images(imgnum+increment, handles, 0);
%increment = str2double(get(handles.Inc,'String'));
%imgnum = str2double(get(handles.RightNum,'String'));
%update_right_images(imgnum+increment, handles, 0);


function Extension_Callback(hObject, eventdata, handles)
% hObject    handle to Extension (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Extension as text
%        str2double(get(hObject,'String')) returns contents of Extension as a double
imgnum = str2double(get(handles.LeftNum,'String'));
update_left_images(imgnum, handles, 0);
imgnum = str2double(get(handles.RightNum,'String'));
update_right_images(imgnum, handles, 0);


% --- Executes on button press in LeftUp.
function LeftUp_Callback(hObject, eventdata, handles)
% hObject    handle to LeftUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
increment = str2double(get(handles.Inc,'String'));
%imgnum = str2double(get(handles.LeftNum,'String'));
%update_left_images(imgnum+increment, handles, 0);
% decrement left sequence
imgnum = str2double(get(handles.LeftNum,'String'));
if get(handles.LeftReverse,'Value')
  update_left_images(imgnum+increment, handles, 0);
else
  update_left_images(imgnum-increment, handles, 0);
end



% --- Executes on button press in LeftDown.
function LeftDown_Callback(hObject, eventdata, handles)
% hObject    handle to LeftDown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
increment = str2double(get(handles.Inc,'String'));
%imgnum = str2double(get(handles.LeftNum,'String'));
%update_left_images(imgnum-increment, handles, 0);
% increment left sequence
imgnum = str2double(get(handles.LeftNum,'String'));
if get(handles.LeftReverse,'Value')
  update_left_images(imgnum-increment, handles, 0);
else
  update_left_images(imgnum+increment, handles, 0);      
end
      

% --- Executes on button press in RightUp.
function RightUp_Callback(hObject, eventdata, handles)
% hObject    handle to RightUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
increment = str2double(get(handles.Inc,'String'));
%imgnum = str2double(get(handles.RightNum,'String'));
%update_right_images(imgnum+increment, handles, 0);
% decrement right sequence
imgnum = str2double(get(handles.RightNum,'String'));
if get(handles.RightReverse,'Value')
  update_right_images(imgnum+increment, handles, 0);      
else    
  update_right_images(imgnum-increment, handles, 0);
end



% --- Executes on button press in RightDown.
function RightDown_Callback(hObject, eventdata, handles)
% hObject    handle to RightDown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
increment = str2double(get(handles.Inc,'String'));
%imgnum = str2double(get(handles.RightNum,'String'));
%update_right_images(imgnum-increment, handles, 0);
% increment right sequence
imgnum = str2double(get(handles.RightNum,'String'));
if get(handles.RightReverse,'Value')
  update_right_images(imgnum-increment, handles, 0);      
else
  update_right_images(imgnum+increment, handles, 0);
end
      

% --- Executes on button press in LeftFlipud.
function LeftFlipud_Callback(hObject, eventdata, handles)
% hObject    handle to LeftFlipud (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of LeftFlipud
imgnum = str2double(get(handles.LeftNum,'String'));
update_left_images(imgnum, handles, 1);


% --- Executes on button press in LeftFliplr.
function LeftFliplr_Callback(hObject, eventdata, handles)
% hObject    handle to LeftFliplr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of LeftFliplr
imgnum = str2double(get(handles.LeftNum,'String'));
update_left_images(imgnum, handles, 1);


% --- Executes on button press in RightFliplr.
function RightFliplr_Callback(hObject, eventdata, handles)
% hObject    handle to RightFliplr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RightFliplr
imgnum = str2double(get(handles.RightNum,'String'));
update_right_images(imgnum, handles, 1);


% --- Executes on button press in RightFlipud.
function RightFlipud_Callback(hObject, eventdata, handles)
% hObject    handle to RightFlipud (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RightFlipud
imgnum = str2double(get(handles.RightNum,'String'));
update_right_images(imgnum, handles, 1);

  
% --- Executes on button press in LeftReverse.
function LeftReverse_Callback(hObject, eventdata, handles)
% hObject    handle to LeftReverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of LeftReverse
imgnum = str2double(get(handles.LeftNum,'String'));
update_left_images(imgnum, handles, 1);

% --- Executes on button press in RightReverse.
function RightReverse_Callback(hObject, eventdata, handles)
% hObject    handle to RightReverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RightReverse
imgnum = str2double(get(handles.RightNum,'String'));
update_right_images(imgnum, handles, 1);


%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function update_left_images(imgnum, handles, reset)
Figdata = get(handles.figure1,'UserData');
LTdata = get(handles.LT,'UserData');
LCdata = get(handles.LC,'UserData');
LBdata = get(handles.LB,'UserData');
FLIPLR = get(handles.LeftFliplr,'Value');
FLIPUD = get(handles.LeftFlipud,'Value');
REVERSE = get(handles.LeftReverse,'Value');

if isempty(LTdata); LTdata.imgnum = -1; end
if isempty(LCdata); LCdata.imgnum = -1; end
if isempty(LBdata); LBdata.imgnum = -1; end

ext = get(handles.Extension,'String');
if isempty(Figdata) || (strcmp(Figdata.ext,ext)==false)
  Figdata.ext = ext;
  Figdata.imgfile = dir(strcat('*',ext));
  Figdata.imgfile = strvcat(Figdata.imgfile.name);
  b = size(Figdata.imgfile,2) - length(ext);
  a = b - 4 +1;
  if a > 0
    Figdata.imgnum = str2num(Figdata.imgfile(:,a:b));
  else
    fprintf(1,'Move to a directory with %s image files\n',ext);
    return;
  end
end

ind = find(Figdata.imgnum == imgnum);
if isempty(ind);
	fprintf(1,'Image number %d not found\n',imgnum);
	return;
end;
ind = ind(1);

if reset == 1
  method = -1;
else
  method = imgnum;
end

if REVERSE
  switch method
   case LTdata.imgnum
    % shift image data rather than read it from hard drive  
    LBdata.I = LCdata.I;
    LCdata.I = LTdata.I;
    LTdata.I = imread(Figdata.imgfile(ind-1,:));
    for ii=1:size(LTdata.I,3)
      if FLIPUD; LTdata.I(:,:,ii) = flipud(LTdata.I(:,:,ii)); end;
      if FLIPLR; LTdata.I(:,:,ii) = fliplr(LTdata.I(:,:,ii)); end;
    end
   case LCdata.imgnum
    % center image is imgnum, therefore leave unchanged
   case LBdata.imgnum
    % shift image data rather than read it from hard drive  
    LTdata.I = LCdata.I;
    LCdata.I = LBdata.I;
    LBdata.I = imread(Figdata.imgfile(ind+1,:));
    for ii=1:size(LBdata.I,3)
      if FLIPUD; LBdata.I(:,:,ii) = flipud(LBdata.I(:,:,ii)); end;
      if FLIPLR; LBdata.I(:,:,ii) = fliplr(LBdata.I(:,:,ii)); end;
    end
   otherwise
    LBdata.I = imread(Figdata.imgfile(ind+1,:));  
    LCdata.I = imread(Figdata.imgfile(ind,:));  
    LTdata.I = imread(Figdata.imgfile(ind-1,:));
    for ii=1:size(LCdata.I,3)
      if FLIPUD; 
	LTdata.I(:,:,ii) = flipud(LTdata.I(:,:,ii));
	LCdata.I(:,:,ii) = flipud(LCdata.I(:,:,ii));
	LBdata.I(:,:,ii) = flipud(LBdata.I(:,:,ii));    
      end
      if FLIPLR; 
	LTdata.I(:,:,ii) = fliplr(LTdata.I(:,:,ii));
	LCdata.I(:,:,ii) = fliplr(LCdata.I(:,:,ii));
	LBdata.I(:,:,ii) = fliplr(LBdata.I(:,:,ii));     
      end
    end
  end % switch
  LBdata.imgnum = Figdata.imgnum(ind+1);
  LCdata.imgnum = Figdata.imgnum(ind);
  LTdata.imgnum = Figdata.imgnum(ind-1);
  
  LBdata.imgfile = Figdata.imgfile(ind+1,:);
  LCdata.imgfile = Figdata.imgfile(ind,:);
  LTdata.imgfile = Figdata.imgfile(ind-1,:);  
else % FORWARD
  switch method
   case LTdata.imgnum
    % shift image data rather than read it from hard drive  
    LBdata.I = LCdata.I;
    LCdata.I = LTdata.I;
    LTdata.I = imread(Figdata.imgfile(ind+1,:));
    for ii=1:size(LTdata.I,3)
      if FLIPUD; LTdata.I(:,:,ii) = flipud(LTdata.I(:,:,ii)); end;
      if FLIPLR; LTdata.I(:,:,ii) = fliplr(LTdata.I(:,:,ii)); end;
    end
   case LCdata.imgnum
    % center image is imgnum, therefore leave unchanged
   case LBdata.imgnum
    % shift image data rather than read it from hard drive  
    LTdata.I = LCdata.I;
    LCdata.I = LBdata.I;
    LBdata.I = imread(Figdata.imgfile(ind-1,:));
    for ii=1:size(LBdata.I,3)
      if FLIPUD; LBdata.I(:,:,ii) = flipud(LBdata.I(:,:,ii)); end;
      if FLIPLR; LBdata.I(:,:,ii) = fliplr(LBdata.I(:,:,ii)); end;
    end
   otherwise  
    LBdata.I = imread(Figdata.imgfile(ind-1,:));  
    LCdata.I = imread(Figdata.imgfile(ind,:));  
    LTdata.I = imread(Figdata.imgfile(ind+1,:));
    for ii=1:size(LCdata.I,3)
      if FLIPUD; 
	LTdata.I(:,:,ii) = flipud(LTdata.I(:,:,ii));
	LCdata.I(:,:,ii) = flipud(LCdata.I(:,:,ii));
	LBdata.I(:,:,ii) = flipud(LBdata.I(:,:,ii));    
      end
      if FLIPLR; 
	LTdata.I(:,:,ii) = fliplr(LTdata.I(:,:,ii));
	LCdata.I(:,:,ii) = fliplr(LCdata.I(:,:,ii));
	LBdata.I(:,:,ii) = fliplr(LBdata.I(:,:,ii));     
      end
    end
  end % switch
  LBdata.imgnum = Figdata.imgnum(ind-1);
  LCdata.imgnum = Figdata.imgnum(ind);
  LTdata.imgnum = Figdata.imgnum(ind+1);
  
  LBdata.imgfile = Figdata.imgfile(ind-1,:);
  LCdata.imgfile = Figdata.imgfile(ind,:);
  LTdata.imgfile = Figdata.imgfile(ind+1,:);
end % if REVERSE

axes(handles.LT); subimage(LTdata.I); axis off;
axes(handles.LC); subimage(LCdata.I); axis off;
axes(handles.LB); subimage(LBdata.I); axis off;

set(handles.LT,'UserData',LTdata);
set(handles.LC,'UserData',LCdata);
set(handles.LB,'UserData',LBdata);

set(handles.LTstr,'String',num2str(LTdata.imgnum));
set(handles.LCstr,'String',num2str(LCdata.imgnum));
set(handles.LBstr,'String',num2str(LBdata.imgnum));

set(handles.LeftNum,'String',num2str(imgnum));



%-------------------------------------------------------------------
function update_right_images(imgnum, handles, reset)
Figdata = get(handles.figure1,'UserData');
RTdata = get(handles.RT,'UserData');
RCdata = get(handles.RC,'UserData');
RBdata = get(handles.RB,'UserData');
FLIPLR = get(handles.RightFliplr,'Value');
FLIPUD = get(handles.RightFlipud,'Value');
REVERSE = get(handles.RightReverse,'Value');

if isempty(RTdata); RTdata.imgnum = -1; end
if isempty(RCdata); RCdata.imgnum = -1; end
if isempty(RBdata); RBdata.imgnum = -1; end

ext = get(handles.Extension,'String');
if isempty(Figdata) || (strcmp(Figdata.ext,ext)==false)
  Figdata.ext = ext;
  Figdata.imgfile = dir(strcat('*',ext));
  Figdata.imgfile = strvcat(Figdata.imgfile.name);
  b = size(Figdata.imgfile,2) - length(ext);
  a = b - 4 +1;
  if a > 0
    Figdata.imgnum = str2num(Figdata.imgfile(:,a:b));
  else
    fprintf(1,'Move to a directory with %s image files\n',ext);
    return;
  end
end

ind = find(Figdata.imgnum == imgnum);
if isempty(ind);
	fprintf(1,'Image number %d not found\n',imgnum);
	return;
end;
ind = ind(1);

if reset == 1
  method = -1;
else
  method = imgnum;
end

if REVERSE
  switch method
   case RTdata.imgnum
    % shift image data rather than read it from hard drive  
    RBdata.I = RCdata.I;
    RCdata.I = RTdata.I;
    RTdata.I = imread(Figdata.imgfile(ind-1,:));
    for ii=1:size(RTdata.I,3)
      if FLIPUD; RTdata.I(:,:,ii) = flipud(RTdata.I(:,:,ii)); end;
      if FLIPLR; RTdata.I(:,:,ii) = fliplr(RTdata.I(:,:,ii)); end;
    end
   case RCdata.imgnum
    % center image is imgnum, therefore leave unchanged
   case RBdata.imgnum
    % shift image data rather than read it from hard drive  
    RTdata.I = RCdata.I;
    RCdata.I = RBdata.I;
    RBdata.I = imread(Figdata.imgfile(ind+1,:));
    for ii=1:size(RBdata.I,3)
      if FLIPUD; RBdata.I(:,:,ii) = flipud(RBdata.I(:,:,ii)); end;
      if FLIPLR; RBdata.I(:,:,ii) = fliplr(RBdata.I(:,:,ii)); end;
    end
   otherwise  
    RBdata.I = imread(Figdata.imgfile(ind+1,:));  
    RCdata.I = imread(Figdata.imgfile(ind,:));  
    RTdata.I = imread(Figdata.imgfile(ind-1,:));
    for ii=1:size(RCdata.I,3)
      if FLIPUD; 
	RTdata.I(:,:,ii) = flipud(RTdata.I(:,:,ii));
	RCdata.I(:,:,ii) = flipud(RCdata.I(:,:,ii));
	RBdata.I(:,:,ii) = flipud(RBdata.I(:,:,ii));    
      end
      if FLIPLR; 
	RTdata.I(:,:,ii) = fliplr(RTdata.I(:,:,ii));
	RCdata.I(:,:,ii) = fliplr(RCdata.I(:,:,ii));
	RBdata.I(:,:,ii) = fliplr(RBdata.I(:,:,ii));     
      end
    end
  end % switch
  RBdata.imgnum = Figdata.imgnum(ind+1);
  RCdata.imgnum = Figdata.imgnum(ind);
  RTdata.imgnum = Figdata.imgnum(ind-1);
  
  RBdata.imgfile = Figdata.imgfile(ind+1,:);
  RCdata.imgfile = Figdata.imgfile(ind,:);
  RTdata.imgfile = Figdata.imgfile(ind-1,:);  
else % FORWARD
  switch method
   case RTdata.imgnum
    % shift image data rather than read it from hard drive  
    RBdata.I = RCdata.I;
    RCdata.I = RTdata.I;
    RTdata.I = imread(Figdata.imgfile(ind+1,:));
    for ii=1:size(RTdata.I,3)
      if FLIPUD; RTdata.I(:,:,ii) = flipud(RTdata.I(:,:,ii)); end;
      if FLIPLR; RTdata.I(:,:,ii) = fliplr(RTdata.I(:,:,ii)); end;
    end
   case RCdata.imgnum
    % center image is imgnum, therefore leave unchanged
   case RBdata.imgnum
    % shift image data rather than read it from hard drive  
    RTdata.I = RCdata.I;
    RCdata.I = RBdata.I;
    RBdata.I = imread(Figdata.imgfile(ind-1,:));
    for ii=1:size(RBdata.I,3)
      if FLIPUD; RBdata.I(:,:,ii) = flipud(RBdata.I(:,:,ii)); end;
      if FLIPLR; RBdata.I(:,:,ii) = fliplr(RBdata.I(:,:,ii)); end;
    end
   otherwise  
    RBdata.I = imread(Figdata.imgfile(ind-1,:));  
    RCdata.I = imread(Figdata.imgfile(ind,:));  
    RTdata.I = imread(Figdata.imgfile(ind+1,:));
    for ii=1:size(RCdata.I,3)
      if FLIPUD; 
	RTdata.I(:,:,ii) = flipud(RTdata.I(:,:,ii));
	RCdata.I(:,:,ii) = flipud(RCdata.I(:,:,ii));
	RBdata.I(:,:,ii) = flipud(RBdata.I(:,:,ii));    
      end
      if FLIPLR; 
	RTdata.I(:,:,ii) = fliplr(RTdata.I(:,:,ii));
	RCdata.I(:,:,ii) = fliplr(RCdata.I(:,:,ii));
	RBdata.I(:,:,ii) = fliplr(RBdata.I(:,:,ii));     
      end
    end
  end % switch
  RBdata.imgnum = Figdata.imgnum(ind-1);
  RCdata.imgnum = Figdata.imgnum(ind);
  RTdata.imgnum = Figdata.imgnum(ind+1);
  
  RBdata.imgfile = Figdata.imgfile(ind-1,:);
  RCdata.imgfile = Figdata.imgfile(ind,:);
  RTdata.imgfile = Figdata.imgfile(ind+1,:);
end % if REVERSE

axes(handles.RT); subimage(RTdata.I); axis off;
axes(handles.RC); subimage(RCdata.I); axis off;
axes(handles.RB); subimage(RBdata.I); axis off;

set(handles.RT,'UserData',RTdata);
set(handles.RC,'UserData',RCdata);
set(handles.RB,'UserData',RBdata);

set(handles.RTstr,'String',num2str(RTdata.imgnum));
set(handles.RCstr,'String',num2str(RCdata.imgnum));
set(handles.RBstr,'String',num2str(RBdata.imgnum));

set(handles.RightNum,'String',num2str(imgnum));

%-------------------------------------------------------------------
function showfig(hObject, eventdata, handles)
% if the user double clicks on an image, open it in a 
% new window for display at maximum resolution
if strcmp(get(handles.figure1,'SelectionType'),'open')
  data = get(gca,'UserData'); % grab current axis data
  if ~isempty(data)
    h = figure;
    imshow(data.I);
    title(sprintf('%d',data.imgnum));
  end
end


%-------------------------------------------------------------------
function advance_sequence(hObject, eventdata, handles)
persistent WHICH_SIDE
LEFT = 108;
RIGHT = 114;
FWD = 102;
BWD = 98;
UP = 117;
DOWN = 100;
if isempty(WHICH_SIDE)
  WHICH_SIDE = LEFT;
end
REVERSE = get(handles.LeftReverse,'Value');
% capture current keystroke and convert it to a numerical value
value = abs(get(hObject,'CurrentCharacter'));
try
  switch value
   case LEFT % L
    WHICH_SIDE = value;
   case RIGHT % R
    WHICH_SIDE = value;
   case {FWD,DOWN} % F,D
    increment = str2double(get(handles.Inc,'String'));  
    if WHICH_SIDE == LEFT
      % increment left sequence
      imgnum = str2double(get(handles.LeftNum,'String'));
      if get(handles.LeftReverse,'Value')
	update_left_images(imgnum-increment, handles, 0);
      else
	update_left_images(imgnum+increment, handles, 0);      
      end
    else
      % increment right sequence
      imgnum = str2double(get(handles.RightNum,'String'));
      if get(handles.RightReverse,'Value')
	update_right_images(imgnum-increment, handles, 0);      
      else
	update_right_images(imgnum+increment, handles, 0);
      end
    end
   case {BWD,UP} % B,U
    increment = str2double(get(handles.Inc,'String'));    
    if WHICH_SIDE == LEFT
      % decrement left sequence
      imgnum = str2double(get(handles.LeftNum,'String'));
      if get(handles.LeftReverse,'Value')
	update_left_images(imgnum+increment, handles, 0);
      else
	update_left_images(imgnum-increment, handles, 0);
      end
    else
      % decrement right sequence
      imgnum = str2double(get(handles.RightNum,'String'));
      if get(handles.RightReverse,'Value')
	update_right_images(imgnum+increment, handles, 0);      
      else    
	update_right_images(imgnum-increment, handles, 0);
      end
    end
   otherwise
    % unrecognized keystroke
  end
catch
  % non-recognizable keystroke
end
