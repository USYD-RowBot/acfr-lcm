function pathlength(TheConfig);
%function pathlength(TheConfig);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-27-2004      rme         Created and written.

global TheJournal;

% shorthand index
fn   = TheJournal.Index.fn;
Xp_i = TheJournal.Index.Xp_i;
Xv_i = TheJournal.Index.Xv_i;

% old vehicle position
oldpos = TheJournal.Pathlen.oldpos;

% current vehicle position
if TheConfig.Estimator.inferenceEif;
  curpos = TheJournal.Eif.mu(Xv_i(Xp_i(1:3)));
else;
  curpos = TheJournal.Ekf.mu(Xv_i(Xp_i(1:3)));
end;

% compute incremental distance traveled
deltaXY  = euclideanDistance(curpos(1:2),oldpos(1:2));
deltaXYZ = euclideanDistance(curpos(1:3),oldpos(1:3));

% add to total distance traveled
TheJournal.Pathlen.pathlen2 = TheJournal.Pathlen.pathlen2 + deltaXY;
TheJournal.Pathlen.pathlen3 = TheJournal.Pathlen.pathlen3 + deltaXYZ;

% store position
TheJournal.Pathlen.oldpos = curpos;
