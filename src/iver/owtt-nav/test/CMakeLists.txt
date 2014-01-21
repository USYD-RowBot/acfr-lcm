#perls_add_executable (test-accumulator test_accumulate_origin_state.cpp)
#perls_target_link_libraries (test-accumulator perls-origin-state perls-owtt-nav
#  perls-common
#  perls-lcmtypes
#  perls-math
#  ${PERLSX_EIGEN3}
#  )

#perls_add_executable (test-index test_index.cpp)
#perls_target_link_libraries (test-index perls-owtt-nav)

#perls_add_executable (test-utils test_utils.cpp)
#perls_target_link_libraries (test-utils)

perls_add_executable (test-origin-state test_origin_state.cpp)
perls_target_link_libraries (test-origin-state perls-origin-state perls-owtt-nav
  ${PERLSX_BOOST}
  ${PERLSX_EIGEN3}
  )

