function [varargout] = random_sample_sphere(Nsamps,radius)
%RANDOM_SAMPLE_SPHERE uniformly samples the surface of a sphere.
%
%  A simple way to randomly (uniform) distribute points on sphere is called
%  the "hypercube rejection method". To apply this to a unit cube at the
%  origin, choose coordinates (x,y,z) each uniformly distributed on the
%  interval [-1,1]. If the length of this vector is greater than 1 then
%  reject it, otherwise normalise it and use it as a sample.
%
%  XYZ = RANDOM_SAMPLE_SPHERE(Nsamps,RADIUS) returns Nsamps from a sphere
%  of radius RADIUS, if RADIUS is not specified it defaults to the unit
%  sphere.  XYZ is a [3 x Nsamps] array.
%  
%  [X,Y,Z] = RANDOM_SAMPLE_SPHERE(Nsamps,RADIUS) returns the [Nsamps x 1]
%  vectors X, Y, and Z individually.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-12-2003      rme         Created and written.

error(nargchk(1,2,nargin));
error(nargoutchk(1,3,nargout));

if ~exist('radius','var')
  radius = 1;
end

XYZ = [];
good_samps = 0;
while good_samps < Nsamps
  n = Nsamps - good_samps;
  
  % sample from uniform distribution [-1,1]
  xyz = 2*rand(3,n)-1;

  % caculate the magnitude of each vector sample
  mag = sum(xyz.*xyz,1).^0.5;

  % keep those samples with magnitude less than 1
  xyz = xyz(:,mag < 1);
  mag = mag(mag < 1);

  % renormalize and store them
  if length(mag) > 0
    XYZ = [XYZ, xyz./repmat(mag,[3 1])];
    good_samps = good_samps + length(mag);
  end
end

if radius ~= 1
  XYZ = radius * XYZ;
end

if nargout == 1
  varargout{1} = XYZ;
else
  varargout{1} = XYZ(1,:)';
  varargout{2} = XYZ(2,:)';
  varargout{3} = XYZ(3,:)';
end
