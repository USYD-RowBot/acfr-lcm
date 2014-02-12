function nim = export2vtk(filename,pose,Xmat,K,res,imgnum,texdir)
% nim = export2vtk(filename,pose,Xmat,K,res,imgnum,texdir)
% 
% INPUTS:
%   filename  is a name for the output files
%   pose      is a 6 x N array of [r;p;h;x;y;z] camera poses x_wci
%   Xmat      is a N x 3 array of structure in the world frame
%   K         is the camera calibration matrix
%   res       is a 2 x 1 array of image pixel resoution, i.e. [nc nr]
%   imgnum    is a N x 1 array of image numbers
%   texdir    is a fullpath string of images to use in texture mapping
%             NOTE: must not contain '~', explictly use /home/user/foo/bar
%
% OUTPUTS:
%   generates filename.vtk a vtk data file with 3D points
%             filename.dat data file with texture map configuration info
%             filename.tcl a tcl script to run in vtk
%
% DATE      WHO     WHAT
%---------  ---     ----------------------------------------
% 20040302  OP      Commented
% 20050510  rme     got rid of OP data structure specific arguments
% 20060113  rme     Commented help file.
% 20060119  rme     Added VTK_DATA environment variable to .tcl file

struc_file = strcat(filename,'.vtk');
npoints = vtk_structure_out(Xmat,struc_file);

tex_file = strcat(filename,'.dat');
ncam = generate_texture_config(pose,K,res,imgnum,texdir,tex_file)

tcl_file = strcat(filename,'.tcl');

% open tcl file and setup filenames for vtk functions
fid = fopen(tcl_file,'w');

fprintf(fid,'set $VTK_DATA/struc_file %s\n',struc_file);
fprintf(fid,'set $VTK_DATA/tex_file %s\n',tex_file);

fprintf(fid,'source sfm_mesh.tcl\n');
fprintf(fid,'source texture_map.tcl\n');

fclose(fid);
