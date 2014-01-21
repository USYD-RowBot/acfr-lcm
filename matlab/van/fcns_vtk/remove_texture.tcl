for {set i 1} {$i < $Nimages} {incr i} {
    ren1 RemoveActor clipActor$i
    points$i Delete
    normals$i Delete
    planes$i Delete
    pngReader$i Delete
    tex$i Delete
    clipper$i Delete
    cam$i Delete
    clipMapper$i Delete
    clipActor$i Delete
}

