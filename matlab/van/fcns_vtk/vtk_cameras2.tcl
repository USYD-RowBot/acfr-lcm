vtkProjectedTexture cam10
   cam10 SetPosition -0.220 4.304 16.617
   cam10 SetFocalPoint -0.265 4.350 17.614
   cam10 SetUp -0.181 -0.983 0.038
   cam10 SetAspectRatio 0.749 0.594 1.000
vtkPlanes planes10
   planes10 SetFrustumPlanes 0.1560 -0.0307 -0.0502 1.0000 -0.1564 0.0262 -0.0690 1.0000 -0.0197 -0.1166 -0.0303 1.0000 0.1369 0.6816 -0.2349 1.0000 0.0017 -0.0017 -0.0374 1.0000 -0.0004 0.0004 0.0080 -1.0000 
vtkPNGReader pngReader10
   pngReader10 SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/test0010.png"
vtkTexture tex10
   tex10 SetInput [pngReader10 GetOutput]
   tex10 SetRepeat 0
vtkClipPolyData clipper10
   clipper10 SetInput [meshNormals GetOutput]
   clipper10 SetClipFunction planes10
vtkPolyDataMapper clip10Mapper
   clip10Mapper SetInput [clipper10 GetOutput]
vtkActor clip10Actor
   clip10Actor SetMapper clip10Mapper
   clip10Actor SetTexture tex10
ren1 AddActor clip10Actor
vtkProjectedTexture cam11
   cam11 SetPosition -0.385 4.680 16.598
   cam11 SetFocalPoint -0.383 4.732 17.597
   cam11 SetUp 0.212 -0.976 0.050
   cam11 SetAspectRatio 0.749 0.594 1.000
vtkPlanes planes11
   planes11 SetFrustumPlanes 0.1656 0.0326 -0.0656 1.0000 -0.1459 -0.0345 -0.0539 1.0000 0.0240 -0.1125 -0.0280 1.0000 -0.1807 0.8154 -0.2944 1.0000 -0.0001 -0.0019 -0.0374 1.0000 0.0000 0.0004 0.0080 -1.0000 
vtkPNGReader pngReader11
   pngReader11 SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/test0011.png"
vtkTexture tex11
   tex11 SetInput [pngReader11 GetOutput]
   tex11 SetRepeat 0
vtkClipPolyData clipper11
   clipper11 SetInput [meshNormals GetOutput]
   clipper11 SetClipFunction planes11
vtkPolyDataMapper clip11Mapper
   clip11Mapper SetInput [clipper11 GetOutput]
vtkActor clip11Actor
   clip11Actor SetMapper clip11Mapper
   clip11Actor SetTexture tex11
ren1 AddActor clip11Actor
