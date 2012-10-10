# This example demonstrates how to use 2D Delaunay triangulation.
# We create a fancy image of a 2D Delaunay triangulation. Points are 
# randomly generated.


# first we load in the standard vtk packages into tcl
package require vtk
package require vtkinteraction
package require vtktesting
 
# Create reader
vtkUnstructuredGridReader reader
    reader SetFileName "/home/opizarro/thesis/code/vtk20040110/sfm_data.vtk"

#vtkUnstructuredGrid ugrid
#ugrid [SetOutput reader]
[reader GetOutput] UpdateData


# Create a polydata with the points we just created.
vtkPolyData profile
profile SetPoints [[reader GetOutput] GetPoints]

# Perform a 2D Delaunay triangulation on them.
#
vtkDelaunay2D del
    del SetInput profile
    del SetTolerance 0.001

#    eval [meshActor GetProperty] SetColor .3 .3 .4

# load in texture map for one camera
#vtkTIFFReader tifReader
 #  tifReader SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/PRG.20020827.21512676.0010.tif"
#tifReader SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/GrayProcessed_16/PXG.20020827.21512676.0010.tif"
#vtkTexture tex10
#   tex10 SetInput [tifReader GetOutput]


vtkPNGReader pngReader10
 #  tifReader SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/PRG.20020827.21512676.0010.tif"
pngReader10 SetFileName "/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/test0010.png"
vtkTexture tex10png
   tex10png SetInput [pngReader10 GetOutput]
   tex10png SetRepeat 0

# setup projector (camera for texture)

vtkProjectedTexture cam10
   cam10 SetPosition -0.220 4.304 16.617
   cam10 SetFocalPoint -0.265 4.350 17.614
   cam10 SetUp -0.181 -0.983 0.038

   cam10 SetAspectRatio 0.749 0.594 1.000
cam10 SetInput [del GetOutput]





vtkPolyDataMapper mapMesh
    mapMesh SetInput [cam10 GetOutput]
vtkActor meshActor
    meshActor SetMapper mapMesh

meshActor SetTexture tex10png

# Create graphics objects
# Create the rendering window, renderer, and interactive renderer
vtkRenderer ren1
vtkRenderWindow renWin
    renWin AddRenderer ren1
vtkRenderWindowInteractor iren
    iren SetRenderWindow renWin

# Add the actors to the renderer, set the background and size


ren1 AddActor meshActor
ren1 SetBackground 1 1 1
renWin SetSize 150 150

# render the image
#
iren AddObserver UserEvent {wm deiconify .vtkInteract}
[ren1 GetActiveCamera] Zoom 1.5
iren Initialize

# prevent the tk window from showing up then start the event loop
wm withdraw .
