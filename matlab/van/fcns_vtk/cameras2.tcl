vtkPoints points10
   points10 InsertNextPoint 0.114 4.564 18.487
   points10 InsertNextPoint 0.566 4.739 17.994
   points10 InsertNextPoint -0.306 4.346 17.982
   points10 InsertNextPoint 0.420 4.164 18.014
   points10 InsertNextPoint 0.566 4.739 17.994
   points10 InsertNextPoint -0.065 4.802 23.979
vtkFloatArray normals10
   normals10 SetNumberOfComponents 3
   normals10 InsertNextTuple3 0.032 -0.043 -0.999
   normals10 InsertNextTuple3 0.919 -0.243 -0.311
   normals10 InsertNextTuple3 -0.896 0.213 -0.390
   normals10 InsertNextTuple3 -0.226 -0.941 -0.252
   normals10 InsertNextTuple3 0.244 0.917 -0.317
   normals10 InsertNextTuple3 -0.032 0.043 0.999
vtkPlanes planes10
   planes10 SetPoints points10
   planes10 SetNormals normals10
vtkPNGReader pngReader10
   pngReader10 SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/test0010.png"
vtkTexture tex10
   tex10 SetInput [pngReader10 GetOutput]
   tex10 SetRepeat 0
vtkClipPolyData clipper10
   clipper10 SetInput [meshNormals GetOutput]
   clipper10 SetClipFunction planes10
   clipper10 InsideOutOn
vtkProjectedTexture cam10
   cam10 SetPosition 0.162 4.500 16.989
   cam10 SetFocalPoint 0.130 4.543 17.988
   cam10 SetUp -0.245 -0.969 0.034
   cam10 SetAspectRatio 0.749 0.594 1.000
   cam10 SetInput [clipper10 GetOutput]
vtkPolyDataMapper clip10Mapper
   clip10Mapper SetInput [cam10 GetOutput]
vtkActor clip10Actor
   clip10Actor SetMapper clip10Mapper
   clip10Actor SetTexture tex10
ren1 AddActor clip10Actor
vtkPoints points11
   points11 InsertNextPoint 0.050 4.953 18.476
   points11 InsertNextPoint 0.369 5.279 17.957
   points11 InsertNextPoint -0.283 4.581 17.997
   points11 InsertNextPoint 0.458 4.693 17.982
   points11 InsertNextPoint 0.369 5.279 17.957
   points11 InsertNextPoint 0.120 5.204 23.970
vtkFloatArray normals11
   normals11 SetNumberOfComponents 3
   normals11 InsertNextTuple3 -0.013 -0.046 -0.999
   normals11 InsertNextTuple3 0.921 0.123 -0.369
   normals11 InsertNextTuple3 -0.930 -0.155 -0.332
   normals11 InsertNextTuple3 0.140 -0.960 -0.243
   normals11 InsertNextTuple3 -0.147 0.934 -0.326
   normals11 InsertNextTuple3 0.013 0.046 0.999
vtkPlanes planes11
   planes11 SetPoints points11
   planes11 SetNormals normals11
vtkPNGReader pngReader11
   pngReader11 SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/test0011.png"
vtkTexture tex11
   tex11 SetInput [pngReader11 GetOutput]
   tex11 SetRepeat 0
vtkClipPolyData clipper11
   clipper11 SetInput [meshNormals GetOutput]
   clipper11 SetClipFunction planes11
   clipper11 InsideOutOn
vtkProjectedTexture cam11
   cam11 SetPosition 0.031 4.885 16.978
   cam11 SetFocalPoint 0.043 4.930 17.977
   cam11 SetUp 0.149 -0.988 0.043
   cam11 SetAspectRatio 0.749 0.594 1.000
   cam11 SetInput [clipper11 GetOutput]
vtkPolyDataMapper clip11Mapper
   clip11Mapper SetInput [cam11 GetOutput]
vtkActor clip11Actor
   clip11Actor SetMapper clip11Mapper
   clip11Actor SetTexture tex11
ren1 AddActor clip11Actor
