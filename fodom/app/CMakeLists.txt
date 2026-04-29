add_executable(visualize        app/visualize.cpp)
add_executable(benchmark_match  app/benchmark_match.cpp)
add_executable(benchmark_icp    app/benchmark_icp.cpp)
add_executable(benchmark_synth  app/benchmark_synth.cpp)
add_executable(icp_viewer       app/icp_viewer.cpp)
add_executable(debug_viewer     app/debug_viewer.cpp)

target_link_libraries(visualize       fodom polyscope)
target_link_libraries(benchmark_match fodom)
target_link_libraries(benchmark_icp   fodom polyscope)
target_link_libraries(benchmark_synth fodom polyscope)
target_link_libraries(icp_viewer      fodom polyscope)
target_link_libraries(debug_viewer    fodom polyscope)

install(TARGETS
    visualize benchmark_match benchmark_icp benchmark_synth icp_viewer debug_viewer
  DESTINATION bin
)