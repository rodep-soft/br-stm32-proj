# Run after CubeMX2 has created the executable and configured its libraries.
# This hook is selected by the Makefile, leaving all generated files untouched.
cmake_language(EVAL CODE "cmake_language(DEFER CALL include [[${CMAKE_CURRENT_LIST_DIR}/application.cmake]])")
