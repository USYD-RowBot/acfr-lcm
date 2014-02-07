# EXTERNAL DEPS
perlsx_boost ()
perlsx_eigen3 ()
perlsx_gsl ()

set (SRC
    index.h             index.cpp
    estimator.h         estimator.cpp
    estimator_info.h    estimator_info.cpp
    estimator_kalman.h  estimator_kalman.cpp
    meas.h              meas.cpp
    meas_sensors.h      meas_sensors.cpp
    meas_queue.h        meas_queue.cpp
    process_model.h     process_model.cpp
    utils.h             utils.cpp
    )
perls_add_library (perls-owtt-nav ${SRC})
perls_target_link_libraries (perls-owtt-nav 
  ${PERLSX_BOOST}
  ${PERLSX_EIGEN3}
  ${PERLSX_GSL}
  )

perls_add_library (perls-origin-state origin_state.h origin_state.cpp)
perls_target_link_libraries (perls-origin-state
  perls-lcmtypes
  ${PERLSX_LCM}
  ${PERLSX_EIGEN3}
  )
