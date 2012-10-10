macro (perlsx_pcl)
  #perlsx_find_package (PCL "PCL not found; on ubuntu `sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl && sudo apt-get install libpcl-1.4-all-dev`")
  perlsx_find_package (PCL "PCL not found; on ubuntu need to install from souce for the time being" ${ARGN})
  mark_as_advanced (
    PCL_INCLUDE_DIRS
    PCL_LIBRARIES
    PCL_LIBRARY_DIRS
    PCL_VERSION
    PCL_COMPONENTS
    PCL_DEFINITIONS
    EIGEN_INCLUDE_DIRS
    FLANN_INCLUDE_DIRS       
    FLANN_LIBRARY            
    FLANN_LIBRARY_DEBUG      
    LIBRARY_OUTPUT_PATH      
    PCL_DIR                  
    PCL_SAMPLE_CONSENSUS_INCLUDE_DIR
    QHULL_INCLUDE_DIRS          
    QHULL_LIBRARY               
    QHULL_LIBRARY_DEBUG
    PCL_APPS_INCLUDE_DIR           
    PCL_COMMON_INCLUDE_DIR         
    PCL_FEATURES_INCLUDE_DIR       
    PCL_FILTERS_INCLUDE_DIR        
    PCL_IO_INCLUDE_DIR            
    PCL_KDTREE_INCLUDE_DIR        
    PCL_KEYPOINTS_INCLUDE_DIR     
    PCL_OCTREE_INCLUDE_DIR        
    PCL_PEOPLE_INCLUDE_DIR        
    PCL_RANGE_IMAGE_INCLUDE_DIR   
    PCL_REGISTRATION_INCLUDE_DIR  
    PCL_SAMPLE_CONSENSUS_INCLUDE_DIR
    PCL_SEARCH_INCLUDE_DIR        
    PCL_SEGMENTATION_INCLUDE_DIR  
    PCL_SURFACE_INCLUDE_DIR       
    PCL_TRACKING_INCLUDE_DIR      
    PCL_VISUALIZATION_INCLUDE_DIR
    OPENNI_INCLUDE_DIRS
    OPENNI_LIBRARY
    PCL_SIMULATION_INCLUDE_DIR
    VTK_DIR 
    )
    
  if (PCL_FOUND)
    include_directories (${PCL_INCLUDE_DIRS})
    add_definitions (${PCL_DEFINITIONS})
    add_definitions (-DANDROID=0 -D__PATHCC__=0)
    set (PERLSX_PCL ${PCL_LIBRARIES}) 
  endif ()
endmacro ()
