# This example demonstrates how to use 2D Delaunay triangulation.
# We create a fancy image of a 2D Delaunay triangulation. Points are 
# randomly generated.


# first we load in the standard vtk packages into tcl
package require vtk
package require vtkinteraction
package require vtktesting

# read in 3D points from a vtk data file created by matlab using
# vtk_structure_out.m
 
# Create reader
vtkUnstructuredGridReader reader
#    reader SetFileName "./sfm_data20040202.vtk"
    reader SetFileName "../../test_loop20040214.vtk"
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

# generate normals for mesh
vtkPolyDataNormals meshNormals
   meshNormals SetInput [del GetOutput]



vtkPolyDataMapper mapMesh
   mapMesh SetInput [meshNormals GetOutput]
vtkActor meshActor
   meshActor SetMapper mapMesh
   eval [meshActor GetProperty] SetColor 1 0 0

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
renWin SetSize 300 300

# render the image
#
iren AddObserver UserEvent {wm deiconify .vtkInteract}
[ren1 GetActiveCamera] Zoom 1.5
iren Initialize

# prevent the tk window from showing up then start the event loop
wm withdraw .
