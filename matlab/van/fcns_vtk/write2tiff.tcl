# Save the window to a png file
vtkWindowToImageFilter w2if
w2if SetInput renWin

vtkTIFFWriter wr
wr SetInput [w2if GetOutput]
wr SetFileName "screengrab.tif"
wr Write

# clean up
wr Delete
w2if Delete
