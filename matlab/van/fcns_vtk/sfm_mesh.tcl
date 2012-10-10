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
puts "vtkUnstructuredGridReader"
vtkUnstructuredGridReader reader
#    reader SetFileName "./sfm_data20040202.vtk"
#    reader SetFileName "../../test_loop20040214.vtk"
reader SetFileName $struc_file
#vtkUnstructuredGrid ugrid
#ugrid [SetOutput reader]
[reader GetOutput] UpdateData


# Create a polydata with the points we just created.
vtkPolyData profile
profile SetPoints [[reader GetOutput] GetPoints]

# Perform a 2D Delaunay triangulation on them.
#
puts "vtkDelaunay2D"
vtkDelaunay2D del
    del SetInput profile
    del SetTolerance 0.001

# generate normals for mesh
puts "vtkPolyDataNormals"
vtkPolyDataNormals meshNormals
   meshNormals SetInput [del GetOutput]


puts "vtkPolyDataMapper"
vtkPolyDataMapper mapMesh
   mapMesh SetInput [meshNormals GetOutput]
vtkActor meshActor
   meshActor SetMapper mapMesh
   eval [meshActor GetProperty] SetColor 1 0 0

# Create graphics objects
# Create the rendering window, renderer, and interactive renderer
puts "vtkRenderer"
vtkRenderer ren1
vtkRenderWindow renWin
    renWin AddRenderer ren1
vtkRenderWindowInteractor iren
    iren SetRenderWindow renWin

# Add the actors to the renderer, set the background and size

puts "ren1 AddActor"
ren1 AddActor meshActor
ren1 SetBackground 1 1 1
renWin SetSize 1000 985

# render the image
#
puts "iren AddObserver"
iren AddObserver UserEvent {wm deiconify .vtkInteract}
[ren1 GetActiveCamera] Zoom 1.5
iren Initialize

# prevent the tk window from showing up then start the event loop
wm withdraw .
