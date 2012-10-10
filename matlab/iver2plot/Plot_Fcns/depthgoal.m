function Goal = depthgoal(iver_t)

bottom = iver_t.z - iver_t.alt.DTB_Height;
% iver_t.ref.Next_Depth < 0 for DFS and >0 for HFB
% in case of DFS, Goal = iver_t.ref.Next_Depth
% in case of HFB, Goal = bottom1 + iver_t.ref.Next_Depth
ii_hfb = find(iver_t.ref.Next_Depth > 0);
Goal = iver_t.ref.Next_Depth;
Goal(ii_hfb) = bottom(ii_hfb) + iver_t.ref.Next_Depth(ii_hfb);
