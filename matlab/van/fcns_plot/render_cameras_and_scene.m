function render_cameras_and_scene(R,t,X,frame,scale,color,map,u,v,Iorig)

if ~exist('frame','var')
  frame = 2;
end
if ~exist('scale','var')
  scale = 1;
end
if ~exist('color','var')
  color = 'ggmk';
end
if ~exist('map','var') && ~exist('Iorig','var')
  map = colormap('copper');
  u = [];
  v = [];
  Iorig = [];
elseif exist('map','var') && ~exist('Iorig','var')
  u = [];
  v = [];
  Iorig = [];
elseif exist('Iorig','var') && isempty(map)
  map = colormap('gray');
end

if frame == 2
  % transfer scene points to camera frame 2
  X = R*X + repmat(t,[1 size(X,2)]);
end

render_cameras(R,t,frame,scale,color);
hold on;
if isempty(Iorig)
  render_scene(X(1,:),X(2,:),X(3,:),map);
else
  render_scene(X(1,:),X(2,:),X(3,:),map,u,v,Iorig);
end
hold off;
axis equal; axis tight;
