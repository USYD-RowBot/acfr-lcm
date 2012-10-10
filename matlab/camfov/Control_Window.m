function []=Control_Window(user)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2010-08-20      ak          Created from camfov

scrsz = get(gcf,'Position');
plot_height = scrsz(4)-200; plot_width = scrsz(3)-100;

isswapped = 0;
no_bg = [.8 .8 .8];

%% Place the control GUI components.
% FOV            
fov_text_width = 140; fov_edit_width = 50;
uicontrol('Style','text','pos',[10,40,fov_text_width,15],...
          'String','Along track FOV','Backgroundcolor',no_bg);
uicontrol('Style','text','pos',[10,20,fov_text_width,15],...
          'String','Cross track FOV','Backgroundcolor',no_bg);
hedit_along_fov = uicontrol('Style','edit',...
                'pos',[10+fov_text_width,40,fov_edit_width,15],...
                'String',num2str(round(user.FOV_along*RTOD*10)/10),...
                'Callback',{@edit_along_fov_Callback});
hedit_cross_fov = uicontrol('Style','edit',...
                'pos',[10+fov_text_width,20,fov_edit_width,15],...
                'String',num2str(round(user.FOV_cross*RTOD*10)/10),...
                'Callback',{@edit_cross_fov_Callback});
hcheck_swap    = uicontrol('Style','checkbox',...
                'pos',[30,60,60,25],...
                'String','swap',...
                'Callback',{@check_swap_fov});
hbutton_reset  = uicontrol('Style','pushbutton',...
                'pos',[100,60,100,25],...
                'String','Reset FOV',...
                'Callback',{@button_reset_fov});
            
% range setting
posx = 10+fov_text_width+fov_edit_width+60;
range_edit_width = 40;
uicontrol('Style','text', 'pos',[posx,80,40,15],'String','offset','Backgroundcolor',no_bg);
uicontrol('Style','text', 'pos',[posx+50,80,40,15],'String','fps','Backgroundcolor',no_bg);
uicontrol('Style','text', 'pos',[posx+50*2,80,40,15],'String','speed','Backgroundcolor',no_bg);
uicontrol('Style','text', 'pos',[posx+50*3,80,60,15],'String','xspace','Backgroundcolor',no_bg);
uicontrol('Style','text', 'pos',[posx-40,60,40,15],'String','max','Backgroundcolor',no_bg);
uicontrol('Style','text', 'pos',[posx-40,40,40,15],'String','tick','Backgroundcolor',no_bg);
uicontrol('Style','text', 'pos',[posx-40,20,40,15],'String','min','Backgroundcolor',no_bg);

hedit_offset_max = uicontrol('Style','edit','pos',[posx,60,range_edit_width,15],...
                  'String',num2str(user.offset_max),'Callback',{@edit_offset_max_Callback});
hedit_offset_tick = uicontrol('Style','edit','pos',[posx,40,range_edit_width,15],...
                  'String',num2str(user.offset_tick),'Callback',{@edit_offset_tick_Callback});
hedit_offset_min = uicontrol('Style','edit','pos',[posx,20,range_edit_width,15],...
                  'String',num2str(user.offset_min),'Callback',{@edit_offset_min_Callback});
hedit_fps_max = uicontrol('Style','edit','pos',[posx+50,60,range_edit_width,15],...
                  'String',num2str(user.fps_max),'Callback',{@edit_fps_max_Callback});
hedit_fps_tick = uicontrol('Style','edit','pos',[posx+50,40,range_edit_width,15],...
                  'String',num2str(user.fps_tick),'Callback',{@edit_fps_tick_Callback});
hedit_fps_min = uicontrol('Style','edit','pos',[posx+50,20,range_edit_width,15],...
                  'String',num2str(user.fps_min),'Callback',{@edit_fps_min_Callback});
hedit_vel_max = uicontrol('Style','edit','pos',[posx+50*2,60,range_edit_width,15],...
                  'String',num2str(user.vel_max),'Callback',{@edit_vel_max_Callback});
hedit_vel_tick = uicontrol('Style','edit','pos',[posx+50*2,40,range_edit_width,15],...
                  'String',num2str(user.vel_tick),'Callback',{@edit_vel_tick_Callback});
hedit_vel_min = uicontrol('Style','edit','pos',[posx+50*2,20,range_edit_width,15],...
                  'String',num2str(user.vel_min),'Callback',{@edit_vel_min_Callback});
hedit_xspace_max = uicontrol('Style','edit','pos',[posx+50*3,60,range_edit_width,15],...
                  'String',num2str(user.xspace_max),'Callback',{@edit_xspace_max_Callback}); 
hedit_xspace_tick = uicontrol('Style','edit','pos',[posx+50*3,40,range_edit_width,15],...
                  'String',num2str(user.xspace_tick),'Callback',{@edit_xspace_tick_Callback});
hedit_xspace_min = uicontrol('Style','edit','pos',[posx+50*3,20,range_edit_width,15],...
                  'String',num2str(user.xspace_min),'Callback',{@edit_xspace_min_Callback});
   

% select type
posx = posx+50*3+70;
htext          = uicontrol('Style','text','String','Select...',...
                'Position',[posx,30,60,20],'Backgroundcolor',no_bg);
hpopup         = uicontrol('Style','popupmenu',...
                'String',{'fps fix (along)','offset fix (along)','vel fix (along)','cross track'},...
                'Position',[posx,0,200,35],...
                'Callback',{@popup_menu_Callback});

% slider                 
label_width=60;
hslide         = uicontrol('Style','slider',...
                'String','slide to select',...
                'Position',[posx,60,200,25],...
                'Max',user.slider_max,...
                'Min',user.slider_min,...
                'Value',user.slider_val,...
                'SliderStep',[0.02 0.1],...
                'Callback',{@slide_Callback});            
uicontrol('style','text','pos',[posx,85,label_width,15],...
          'horizontalalignment','l','String','min','Backgroundcolor',no_bg); % max
uicontrol('style','text','pos',[posx+170,85,label_width,15],...
          'horizontalalignment','l','String','max','Backgroundcolor',no_bg); % min
htext_slider_val = uicontrol('style','text','pos',[posx+70,85,label_width,15],...
                  'horizontalalignment','c','Backgroundcolor',[.9 .9 .9]); % val

% draw button
posx = posx+50*3+70;
hdraw = uicontrol('Style','pushbutton','String','Redraw',...
       'Position',[posx,30,80,50],'Callback',{@drawbutton_Callback});

% main plot
ha = axes('Units','Pixels','Position',[50,60+user.ctrlbar_h,plot_width,plot_height]);


%% GUI options for plot
%align([hdraw,htext,hpopup],'Center','None');

% Change units to normalized so components resize automatically.
% set([1,ha,hdraw,htext],'Units','normalized');

% To enalbe callback when dragging slide bar
set(1, 'WindowButtonMotionFcn',@slide_Callback);

% default plot is "fixed fps"
draw_opt = 'fps';
target_fhandle = @draw_fix_fps;
feval(target_fhandle)
set(htext_slider_val,'string',['(',num2str(user.fps),')']);

% Move the GUI to the center of the screen.
movegui(1,'center')
% Make the GUI visible.
set(1,'Visible','on');

%%  CALLBACK FUNCTIONS (uicontrol)
% popup menu (list) callbacks
function popup_menu_Callback(source,eventdata)
    % Determine the selected data set.
    str = get(source, 'String');
    val = get(source,'Value');

    % Set current data to the selected data set.
    switch str{val};
        case 'offset fix (along)'
            target_fhandle = @draw_fix_offset;
        case 'cross track'
            target_fhandle = @draw_vertical_overlap;
        case 'fps fix (along)'
            target_fhandle = @draw_fix_fps;
        case 'vel fix (along)'
            target_fhandle = @draw_fix_vel;
    end
end

% Push button callbacks. Draw now.
function drawbutton_Callback(source,eventdata)
    % Re draw !
    scrsz = get(gcf,'Position');
	plot_height = scrsz(4)-200; plot_width = scrsz(3)-100;
    user = get(gcf,'Userdata');
    axes('Units','Pixels','Position',[50,60+user.ctrlbar_h,plot_width,plot_height]);
    feval(target_fhandle);
end

function slide_Callback(source,eventdata)
% max_val=get(h,'Max');
    slider_val=get(hslide,'Value');
    slider_val = round(slider_val*10)/10;
    set(hslide,'Value',slider_val);

    switch draw_opt
        case 'offset'
            user.offset = slider_val;
            if (user.offset > user.offset_max || user.offset < user.offset_min)
                user.offset = user.offset_max;
            end
            set(htext_slider_val,'string',['(',num2str(user.offset),')']);
            set(hslide,'Value',user.offset);                        
            set(hslide,'Max',user.offset_max);
            set(hslide,'Min',user.offset_min);
        case 'fps'
            user.fps = slider_val;
            if (user.vel > user.vel_max || user.vel < user.vel_min)
                user.vel = user.vel_max;
            end
            set(htext_slider_val,'string',['(',num2str(user.fps),')']);
            set(hslide,'Value',user.fps);            
            set(hslide,'Max',user.fps_max);
            set(hslide,'Min',user.fps_min);
            
        case 'vel'
            user.vel=slider_val;
            if (user.vel > user.vel_max || user.vel < user.vel_min)
                user.vel = user.vel_max;
            end
            set(htext_slider_val,'string',['(',num2str(user.vel),')']);
            set(hslide,'Value',user.vel);            
            set(hslide,'Max',user.vel_max);
            set(hslide,'Min',user.vel_min);
    end
    drawnow;
    feval(target_fhandle);

end

% fov
function edit_along_fov_Callback(source,eventdata)
    new_along_fov = str2double(get(hedit_along_fov,'String'));
    user.FOV_along = new_along_fov*DTOR;
    feval(target_fhandle);
end

function edit_cross_fov_Callback(source,eventdata)
    new_cross_fov = str2double(get(hedit_cross_fov,'String'));
    user.FOV_cross = new_cross_fov*DTOR;
    feval(target_fhandle);
end

function check_swap_fov(source,eventdata)
% Swap FOV value
    check_st = get(hcheck_swap,'Value');
    if (check_st)
        temp_fov = user.FOV_along;
        user.FOV_along = user.FOV_cross;
        user.FOV_cross = temp_fov;
        isswapped = 1;
    elseif (isswapped)
        temp_fov = user.FOV_along;
        user.FOV_along = user.FOV_cross;
        user.FOV_cross = temp_fov;
        isswapped = 0;
    end
    set(hedit_along_fov,'String',num2str(round(user.FOV_along*RTOD*10)/10));
    set(hedit_cross_fov,'String',num2str(round(user.FOV_cross*RTOD*10)/10));
    feval(target_fhandle);
end

function button_reset_fov(source,eventdata)
% Reset the FOV value back to the one in mfile    
    user.FOV_along = 41.1086*DTOR;
    user.FOV_cross = 33.5269*DTOR;
    set(hedit_along_fov,'String',num2str(round(user.FOV_along*RTOD*10)/10));
    set(hedit_cross_fov,'String',num2str(round(user.FOV_cross*RTOD*10)/10));
    feval(target_fhandle);
end

% range: offset
function edit_offset_min_Callback(source,eventdata)
    user.offset_min = str2double(get(hedit_offset_min,'String'));
    user.offset_range = user.offset_min:user.offset_tick:user.offset_max;
end

function edit_offset_max_Callback(source,eventdata)
    user.offset_max = str2double(get(hedit_offset_max,'String'));
    user.offset_range = user.offset_min:user.offset_tick:user.offset_max;
end

function edit_offset_tick_Callback(source,eventdata)
    user.offset_tick = str2double(get(hedit_offset_tick,'String'));
    user.offset_range = user.offset_min:user.offset_tick:user.offset_max;
end

% range: fps
function edit_fps_min_Callback(source,eventdata)
    user.fps_min = str2double(get(hedit_fps_min,'String'));
    user.fps_range = user.fps_min:user.fps_tick:user.fps_max;
end

function edit_fps_max_Callback(source,eventdata)
    user.fps_max = str2double(get(hedit_fps_max,'String'));
    user.fps_range = user.fps_min:user.fps_tick:user.fps_max;
end

function edit_fps_tick_Callback(source,eventdata)
    user.fps_tick = str2double(get(hedit_fps_tick,'String'));
    user.fps_range = user.fps_min:user.fps_tick:user.fps_max;
end

% range: vel
function edit_vel_min_Callback(source,eventdata)
    user.vel_min = str2double(get(hedit_vel_min,'String'));
    user.vel_range = user.vel_min:user.vel_tick:user.vel_max;
end

function edit_vel_max_Callback(source,eventdata)
    user.vel_max = str2double(get(hedit_vel_max,'String'));
    user.vel_range = user.vel_min:user.vel_tick:user.vel_max;
end

function edit_vel_tick_Callback(source,eventdata)
    user.vel_tick = str2double(get(hedit_vel_tick,'String'));
    user.vel_range = user.vel_min:user.vel_tick:user.vel_max;
end

% range: xspace
function edit_xspace_min_Callback(source,eventdata)
    user.xspace_min = str2double(get(hedit_xspace_min,'String'));
    user.xspace_range = user.xspace_min:user.xspace_tick:user.xspace_max;
end

function edit_xspace_max_Callback(source,eventdata)
    user.xspace_max = str2double(get(hedit_xspace_max,'String'));
    user.xspace_range = user.xspace_min:user.xspace_tick:user.xspace_max;
end

function edit_xspace_tick_Callback(source,eventdata)
    user.xspace_tick = str2double(get(hedit_xspace_tick,'String'));
    user.xspace_range = user.xspace_min:user.xspace_tick:user.xspace_max;
end

%% DRAW FUNCTIONS
function draw_fix_fps
% draw the change of other parameter with fixed fps
% plot overlap - vel graph with different offset
    user.FOV = user.FOV_along;
    legend_str = [];
    draw_opt = 'fps';
    dt  = 1/(user.fps);
    h_set = 2*user.offset_range*tan(user.FOV/2);
    for ii=1:length(h_set)
        legend_str{ii} = ['offset=',num2str(user.offset_range(ii))];
        dx = h_set(ii)-user.vel_range*dt;
        overlap = dx/h_set(ii)*100;
        plot(user.vel_range,overlap,'Color',user.color_set(mod(ii,length(user.color_set))+1));
        hold on;
    end
    axis([user.vel_min,user.vel_max,user.overlap_min,user.overlap_max]);
    hold off; grid on;
    xlabel('velocity [m/s]'); ylabel('overlap ratio [%]');
    title(['overlap ratio w.r.t. velocity with fixed fps (fps = ',num2str(user.fps),')']);
    legend(legend_str,'Location','NorthEastOutside');
    drawnow;
end

function draw_fix_offset
% draw the change of other parameter with fixed offset
% plot overlap - vel graph with different fps
    user.FOV = user.FOV_along;
    legend_str = [];
    draw_opt = 'offset';
    dt_set  = 1./user.fps_range;
    h = 2*user.offset*tan(user.FOV/2);
    for ii=1:length(dt_set)
        legend_str{ii} = ['fps=',num2str(user.fps_range(ii))];
        dx = h-user.vel_range*dt_set(ii);
        overlap = dx./h.*100;
        plot(user.vel_range,overlap,'Color',user.color_set(mod(ii,length(user.color_set))+1));
        hold on;
    end
    axis([user.vel_min,user.vel_max,user.overlap_min,user.overlap_max]);
    hold off; grid on;
    xlabel('velocity [m/s]'); ylabel('overlap ratio [%]');
    title(['overlap ratio w.r.t. velocity with fixed offset (offset = ',num2str(user.offset),')']);
    legend(legend_str,'Location','NorthEastOutside');
    drawnow;
end

function draw_fix_vel
% draw the change of other parameter with fixed velocity
% plot overlap - offset graph with different fps
    user.FOV = user.FOV_along;
    legend_str = [];
    draw_opt = 'vel';
    dt_set  = 1./user.fps_range;
    h_set = 2*user.offset_range*tan(user.FOV/2);
    for ii=1:length(dt_set)
        legend_str{ii} = ['fps=',num2str(user.fps_range(ii))];
        dx = h_set-user.vel*dt_set(ii);
        overlap = dx./h_set.*100;
        plot(user.offset_range,overlap,'Color',user.color_set(mod(ii,length(user.color_set))+1));
        hold on;
    end
    axis([user.offset_min,user.offset_max, user.overlap_min,user.overlap_max]);
    hold off; grid on;
    xlabel('Offset [m]'); ylabel('overlap ratio [%]');
    title(['overlap ratio w.r.t. offset with fixed velocity (vel = ',num2str(user.vel),')']);
    legend(legend_str,'Location','NorthEastOutside');
    drawnow;
end

function draw_vertical_overlap
% draw the change of overlap and cross-track-line distance
% plot overlap - (cross-track-line) distance graph with different fps
    user.FOV = user.FOV_cross;
    legend_str = [];
    draw_opt = 'cross';
    h_set = 2*user.offset_range*tan(user.FOV/2);
    for ii=1:length(user.offset_range)
        legend_str{ii} = ['offset=',num2str(user.offset_range(ii))];
        dx = h_set(ii)-user.xspace_range;
        overlap = dx./h_set(ii).*100;
        plot(user.xspace_range,overlap,'Color',user.color_set(mod(ii,length(user.color_set))+1));
        hold on;
    end
    axis([user.xspace_min,user.xspace_max,user.overlap_min,user.overlap_max]);
    hold off; grid on;
    xlabel('Cross track line space [m]'); ylabel('overlap ratio [%]');
    title(['overlap ratio w.r.t. cross-track-line-space']);
    legend(legend_str,'Location','NorthEastOutside');
    drawnow;
end

end % end of the function camfov
