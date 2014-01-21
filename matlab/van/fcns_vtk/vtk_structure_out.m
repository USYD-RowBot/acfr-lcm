function npoints = vtk_structure_out(Xmat,filename);

fid = fopen(filename,'w');
npoints = size(Xmat,1);

fprintf(fid,'# vtk DataFile Version 2.0\n');
fprintf(fid,'Triangulated structure\n ');
fprintf(fid,'ASCII\n');
fprintf(fid,'DATASET UNSTRUCTURED_GRID\n');
fprintf(fid,'POINTS %i double\n',npoints);

for k = 1:npoints
  fprintf(fid,'%.3f %.3f %.3f\n',Xmat(k,1),Xmat(k,2),Xmat(k,3));
end

fclose(fid);
