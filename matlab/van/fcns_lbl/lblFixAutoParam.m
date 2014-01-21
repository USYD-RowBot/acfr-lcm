function Param = lblFixAutoParam(nav_t,TheConfig);
%function Param = lblFixAutoParam(nav_t,TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-06-2006      rme         Created and written.
%    02-03-2006      rme         Updated to work with new nav_t LBL organization.

Param.owtt_rovtime  = nav_t.LBL.rovtime;
Param.owtt          = nav_t.LBL.owtt;
Param.depth_rovtime = nav_t.PARO.rovtime;
Param.depth         = nav_t.PARO.depth;
if isfield(nav_t,'PHINS');
  Param.rph_rovtime = nav_t.PHINS.rovtime;
  Param.rph         = [nav_t.PHINS.roll_cooked, nav_t.PHINS.pitch_cooked, nav_t.PHINS.heading_cooked];
elseif isfield(nav_t,'RDI');
  Param.rph_rovtime = nav_t.RDI.rovtime;
  Param.rph         = [nav_t.RDI.roll_cooked, nav_t.RDI.pitch_cooked, nav_t.RDI.heading_cooked];  
end;
Param.x_vs_depth    = TheConfig.SensorXform.PARO.x_vs;
Param.x_vs_ducer    = TheConfig.SensorXform.LBL.x_vs;
Param.btat          = 0e-3 *[1,1,1,1];
Param.bxyz          = [nav_t.LBL.lbl_xponder1(4:6)', ...
		       nav_t.LBL.lbl_xponder2(4:6)', ...
		       nav_t.LBL.lbl_xponder3(4:6)', ...
		       nav_t.LBL.lbl_xponder4(4:6)'];
% correspondence between A,B,C,D and beacon column index
if any(nav_t.LBL.lbl_xponder1);
  Param.abcd = [1,0,0,0];
end;
if any(nav_t.LBL.lbl_xponder2);
  Param.abcd = [1,2,0,0];
end;
if any(nav_t.LBL.lbl_xponder3);
  Param.abcd = [1,2,3,0];
end;
if any(nav_t.LBL.lbl_xponder4);
  Param.abcd = [1,2,3,4];
end;
switch lower(nav_t.LBL.lbl_baseline);
case 'cw';  Param.cwflag = 1;
case 'ccw'; Param.cwflag = -1;
end;
Param.sos = nav_t.LBL.lbl_sos;
