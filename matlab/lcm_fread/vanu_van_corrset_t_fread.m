function corr_mat = vanu_van_corrset_t_fread (filename)
% function mat_struct = vanu_van_corrset_t_fread (filename)
%
% reads fwritten van_corrset_t lcm structure and returns matlab structure
%  input:  filename
%  output: van_corrset_t in matlab structure
%
%  NOTE: before calling this function, make sure you've added jars needed
%
%  import java.io.*;
%  eval(['javaaddpath ','${PERLS_DIR}/third-party/build/lcm-0.5.2/lcm-java/lcm.jar']);
%  eval(['javaaddpath ','${PERLS_DIR}/lib/perllcm.jar']);
%
%  Ayoung Kim, 2011.06.08 

ins = java.io.DataInputStream(java.io.BufferedInputStream(java.io.FileInputStream(filename)));

% read header line
for i=1:8, t = ins.readByte(); end

% read total size of the file
sz = ins.readInt();

corr = perllcm.van_corrset_t(ins);

corr_mat.utime_i = corr.utime_i;
corr_mat.utime_j = corr.utime_j;
corr_mat.z = zeros(5,1);
corr_mat.R = zeros(5,5);
for i=1:5
    corr_mat.z(i) = corr.z(i);
    for j=1:5
        corr_mat.R(i,j) = corr.R(5*(i-1)+j);
    end
end
        
corr_mat.npts = corr.npts;
corr_mat.u1 = zeros(corr_mat.npts,1);
corr_mat.v1 = zeros(corr_mat.npts,1);
corr_mat.u2 = zeros(corr_mat.npts,1);
corr_mat.v2 = zeros(corr_mat.npts,1);

for i=1:corr_mat.npts
    corr_mat.u1(i) = corr.u1(i);
    corr_mat.v1(i) = corr.v1(i);
    corr_mat.u2(i) = corr.u2(i);
    corr_mat.v2(i) = corr.v2(i);
end