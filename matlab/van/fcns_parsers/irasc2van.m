% Script to convert Brendan's IRAS-C mission3_inum1_inum2.txt files
% into ManualPoints-inum1-inum2.dat.  call this script from the 
% directory where the raw IRAS-C files are.

dir_t = dir('*.txt');
for ii=1:length(dir_t);
  [u1,v1,u2,v2,iname1,iname2] = readirasc(dir_t(ii).name);
  ext = '.jpg';
  inum1 = strread(iname1{1},['%d',ext]);
  inum2 = strread(iname2{1},['%d',ext]);
  if inum2 > inum1;
    [inum1,inum2] = deal(inum2,inum1);
    [u1,v1,u2,v2] = deal(u2,v2,u1,v1);
  end;
  fname = sprintf('Manual-%04d-%04d.dat',inum1,inum2);
  fid = fopen(fname,'w');
  fprintf(fid,'%10.4f\t%10.4f\t%10.4f\t%10.4f\n',[u1,v1,u2,v2]');
  fclose(fid);
end;
