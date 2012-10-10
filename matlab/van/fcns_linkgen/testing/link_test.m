linkA_t = link_t;
linkB_t = link_t;
linkC_t = link_t;

linkA_t.vlinks(:) = 0;
linkB_t.vlinks(:) = 0;
linkC_t.vlinks(:) = 0;

%tic;
[linkAB_t,fAi,fAj] = linkhypoth_AB(linkA_t,TheJournal,TheConfig);
%deltaAB = toc
return;

tic;
[linkA_t,fAi,fAj] = linkhypoth_ekfA(linkA_t,TheJournal,TheConfig);
deltaA = toc
tic;
[linkB_t,fBi,fBj] = linkhypoth_eifB(linkB_t,TheJournal,TheConfig);
deltaB = toc
%tic;
%[linkC_t,fCi,fCj] = linkhypoth_eifC(linkC_t,TheJournal,TheConfig);
%deltaC = toc

figure(1);
spy(linkA_t.plinks>0.99);
title('Full Covariance Link Hypothesis');

figure(2);
spy(linkB_t.plinks>0.99);
title('Conditional Covariance Link Hypothesis');

%figure(3);
%spy(linkC_t.plinks>0.99);
%title('Conditional Covariance SEIF Link Hypothesis');
