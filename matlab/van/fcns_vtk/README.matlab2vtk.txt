GENERATING THE VAN POINTCLOUD WITHIN MATLAB
===================================================
1) Run pairwise_pointcloud.m to generate the Points data structure.

2) Run points2vec.m to generate PtsVan and PtsTwo.

3) Run clippoints.m on PtsVan to discard outliers, based upon
   triangulation error tolerances and scene depth constraitns.  You
   may need to add and/or configure the appropriate case statement for
   your data.  This function returns the index ii.

4) Optionally, fit a smooth surface to the clippoints data points
   using: 
   [X,Y,Z,binpop] = pointcloud2mesh(PtsVan.Xw(:,ii),0.05,0.05,1,0.1); 

   Now use the smooth suface to discard additional outliers in the
   pointcloud based upon a tolerance band.  
   
   jj = findOutliers3D(PtsVan.Xw,X,Y,Z,0.25); 
   kk = ii & jj; % keep the intersection
   % plot clipped pointcloud
   fscatter(PtsVan.Xw(1,kk),PtsVan.Xw(2,kk),PtsVan.Xw(3,kk),PtsVan.Xw(3,kk),jet);


EXPORTING MATLAB POINTCLOUD TO VTK
===================================================
1) png_textures.m or png_color_textures.m downsamples images to be
   used in texture map.

2) Convert TheJournal pose data into oscar's pose format
   cpose = rme2op(TheJournal,TheConfig);

3) export2vtk.m creates the 3D points list (.vtk), the
   camera/projector info (.dat) and the script to run in vtk (.tcl)
   export2vtk('myfile',cpose,PtsVan.Xw(:,kk)',K,[1280,1024],imgnum,texdir);

4) Move the textureX file and .tcl .dat .vtk to your data directory in
   a subdir named vtkdata, e.g.
   ls /files1/data/titanic04/van_processed/vtkdata
     texture4/
     titanic04-pts.dat
     titanic04-pts.tcl
     titanic04-pts.vtk

VIEW DATA IN VTK
===================================================
1) cd into fcns_vtk

2) edit vtdata.tcl and set to correct data directory

3) type `vtk` from terminal window to start a VTK session.  
   in the vtk shell run the script: 
   `source vtkdata.tcl`
   `source $VTK_DATA/mydata.tcl`
   `source texture_map.tcl` (if commented out in mydata.tcl)

