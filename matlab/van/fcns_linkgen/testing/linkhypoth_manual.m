function plinks = linkhypoth_manual(plinks,vlinks,imgnum,featureLUT)
  
% simply propose temporal sequence for now
plinks(:) = 0;      
cind = find(featureLUT==imgnum);   % current image index in feature LUT
pind = cind-1;                  % previous image index in feature LUT
plinks(pind,cind) = 1; % only use upper triangular part of matrix to
		       % reduce memory storage since link matrix is symmetric

if false		       
% manually propose spatial links where i have verified that
% there is indeed overlap
switch imgnum
%==============================================
% LEGS: 3N-4S, NORTH TURN #2
%==============================================
% case 575
%  tmp = find(featureLUT==544); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==545); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 576
  tmp = find(featureLUT==543); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==544); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 577
  tmp = find(featureLUT==542); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==543); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==544); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 578
%  tmp = find(featureLUT==541); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==542); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 579
%  tmp = find(featureLUT==541); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 580
%  tmp = find(featureLUT==539); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==540); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 581
%  tmp = find(featureLUT==538); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==539); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==540); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 582
%  tmp = find(featureLUT==537); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==538); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==539); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 583
%  tmp = find(featureLUT==536); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==537); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 584
%  tmp = find(featureLUT==535); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==536); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 585
%  tmp = find(featureLUT==534); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==535); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
% case 586
%  tmp = find(featureLUT==533); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==534); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%  tmp = find(featureLUT==535); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%==============================================
% LEGS: 4S-5N, SOUTH TURN #2
%==============================================
 case 754
  tmp = find(featureLUT==734); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==735); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 755
  tmp = find(featureLUT==733); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==734); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 756
  tmp = find(featureLUT==732); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==733); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 757
  tmp = find(featureLUT==731); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==732); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 758
  tmp = find(featureLUT==730); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==731); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 759
  tmp = find(featureLUT==729); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==730); if ~isempty(tmp); plinks(tmp,cind) = 1; end;  
 case 760
  tmp = find(featureLUT==728); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==729); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 761
  tmp = find(featureLUT==727); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==728); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 762
  tmp = find(featureLUT==726); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==727); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 763
  tmp = find(featureLUT==725); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==726); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
%==============================================
% LEGS: 15N-16S
%==============================================
 case 2767
  tmp = find(featureLUT==2730); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2768
  tmp = find(featureLUT==2729); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2769
  tmp = find(featureLUT==2728); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2770
  tmp = find(featureLUT==2727); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==2726); if ~isempty(tmp); plinks(tmp,cind) = 1; end;  
 case 2771
  tmp = find(featureLUT==2726); if ~isempty(tmp); plinks(tmp,cind) = 1; end;  
  tmp = find(featureLUT==2725); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2772
  tmp = find(featureLUT==2725); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2773
  tmp = find(featureLUT==2724); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2774
  tmp = find(featureLUT==2723); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2775
  tmp = find(featureLUT==2722); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==2721); if ~isempty(tmp); plinks(tmp,cind) = 1; end; 
 case 2776
  tmp = find(featureLUT==2721); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==2720); if ~isempty(tmp); plinks(tmp,cind) = 1; end; 
 case 2777
  tmp = find(featureLUT==2720); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==2719); if ~isempty(tmp); plinks(tmp,cind) = 1; end; 
 case 2778
  tmp = find(featureLUT==2719); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
  tmp = find(featureLUT==2718); if ~isempty(tmp); plinks(tmp,cind) = 1; end; 
 case 2779
  tmp = find(featureLUT==2718); if ~isempty(tmp); plinks(tmp,cind) = 1; end;
 case 2782
  tmp = find(featureLUT==2715); if ~isempty(tmp); plinks(tmp,cind) = 1; end;

 otherwise
  % don't propose any links
end % switch imgnum
end
