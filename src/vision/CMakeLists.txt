# LIBRARIES
perls_add_folder (perls-vision "Build perls-vision library?" ON)

# EXE's
perls_add_folder (autocal "Build camera calibration tool?" OFF)
perls_add_folder (botimg-publish "Build botimg-publish tool?" OFF)
perls_add_folder (camlog-convert "Build image log file conversion tool?" OFF)
perls_add_folder (camlog-export "Build LCM log image export tool?" OFF)
perls_add_folder (camlog-logger "Build LCM image logging tool?" OFF)
perls_add_folder (camlog-player "Build LCM image playback tool?" OFF)
perls_add_folder (clahs "Build contrast-limited histogram equalization command line tool?" OFF)
perls_add_folder (features "Build image feature extraction process?" OFF)
perls_add_folder (ferns-tracker "Build ferns target tracker process?" OFF)
perls_add_folder (siftgpu "Build SiftGPU server/client tools?" OFF)
perls_add_folder (surfgpu-asrl "Build ASRL SurfGPU server/client tools (Autonomous Space Robotics Lab)?" OFF)
perls_add_folder (surfgpu-ocv "Build OpenCV SurfGPU server/client tools?" OFF)

# CAMERA DRIVERS
perls_add_folder (bbxb3 "Build Point Grey Bumblebee XB3 camera driver?" OFF)
perls_add_folder (dc1394-camera "Build general dc1394 camera driver?" OFF)
perls_add_folder (lb3 "Build Point Grey Ladybug3 camera driver?" OFF)
perls_add_folder (prosilica "Build AVT Prosilica GigE camera driver?" OFF)

# ACFR versions
perls_add_folder (acfr-prosilica "Build ACFR AVT Prosilica GigE camera driver?" ON)
perls_add_folder (acfr-cam-logger "Build ACFR LCM image logging tool?" ON)
perls_add_folder (acfr-cam-server-lcm "Build ACFR cam server process?" ON)
perls_add_folder (acfr-cam-server "Build ACFR LCM cam server process?" ON)
perls_add_folder (cam-trigger "Build ACFR cam trigger tool?" ON)
perls_add_folder (cam-viewer "Build ACFR cam viewer tool?" ON)
perls_add_folder (cam-compress "Build ACFR cam compress and send tool?" ON)
