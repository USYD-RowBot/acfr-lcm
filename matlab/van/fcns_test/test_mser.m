%I1 = imread('/files1/data/oc387/Images/oc387-6boulder/ColorAvgProcessed/CAF.20030325.19450748.0358.jpg');
%I2 = imread('/files1/data/oc387/Images/oc387-6boulder/ColorAvgProcessed/CAF.20030325.19461978.0382.jpg');
%I1 = imread('/files1/data/oc387/Images/oc387-6boulder/ColorProcessed/PXC.20030325.19450748.0358.tif');
%I2 = imread('/files1/data/oc387/Images/oc387-6boulder/ColorProcessed/PXC.20030325.19461978.0382.tif');
I1 = imread('/home/files1/data/oc387/Images/oc387-6boulder/ColorAvgProcessed/CAF.20030325.19543071.0543.jpg');
I2 = imread('/home/files1/data/oc387/Images/oc387-6boulder/ColorAvgProcessed/CAF.20030325.19561009.0576.jpg');


I1 = I1(end:-1:1,end:-1:1,:);
figure(1); imshow(I1,'notruesize'); title('I1');
figure(2); imshow(I2,'notruesize'); title('I2');

switch 1
 case 1
  J1 = rgb2gray(I1);
  J2 = rgb2gray(I2);
  
  J1 = im2uint8(J1);
  J2 = im2uint8(J2);
 case 2
  K1 = rgb2hsv(I1);
  K2 = rgb2hsv(I2);
end % switch
figure(3); imshow(J1,'notruesize'); title('J1');
figure(4); imshow(J2,'notruesize'); title('J2');
drawnow;

if false
  [scale,theta,t] = fftreg(J1,J2);
  tform = tform_from_similarity(scale,theta,t,(size(J1)-1)./2);
  Imos = Merge_pair(J1,J2,tform);
  figure(5); imshow(Imos); title('Imos');
end

if true
Nmax = 400;
% run on I1
Nfig = 10;
Q = imresize(J1,0.25,'bicubic');
figure(Nfig); imshow(Q); truesize;
[drmin,drmax] = DRfind(Q);
msermax = mser(drmax,Nmax);
msermin = mser(drmin,Nmax);
show_mser(Q,msermax,'+',Nfig+1);
show_mser(Q,msermin,'-',Nfig+3);
figure(Nfig+1); truesize; figure(Nfig+2); truesize;
figure(Nfig+3); truesize; figure(Nfig+4); truesize;
% run on I2
Nfig = 20;
Q = imresize(J2,0.25,'bicubic');
figure(Nfig); imshow(Q); truesize;
[drmin,drmax] = DRfind(Q);
msermax = mser(drmax,Nmax);
msermin = mser(drmin,Nmax);
show_mser(Q,msermax,'+',Nfig+1);
show_mser(Q,msermin,'-',Nfig+3);
figure(Nfig+1); truesize; figure(Nfig+2); truesize;
figure(Nfig+3); truesize; figure(Nfig+4); truesize;
end
