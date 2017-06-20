add_library(DivideAndPrintResult SHARED DivideAndPrintResult.cc)
add_library(AddAndPrintResult SHARED AddAndPrintResult.cc)

target_link_libraries(AddAndPrintResult
  gazeboComponentTriplet
  GazeboECS)
target_link_libraries(DivideAndPrintResult
  gazeboComponentFraction
  GazeboECS)
