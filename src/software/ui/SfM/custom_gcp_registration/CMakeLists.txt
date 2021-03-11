
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
# The name of my project.
SET(ProjectName ui_openMVG_custom_gcp_registration)
PROJECT(${ProjectName})
# All Source Files here
SET(SourceFiles
GCP_register.cpp
GCP.cpp
GCPList.cpp)
# All non moc header files here
SET(HeaderFiles
GCP_register.hpp
GCP.hpp
GCPList.hpp)
# ==============================================================================
# Add the following files, modules and libraries to the project.
# ==============================================================================
# Files
ADD_EXECUTABLE(${ProjectName}
${SourceFiles}
${HeaderFiles})
# Linking with openMVG Libraries.
TARGET_LINK_LIBRARIES(${ProjectName}
openMVG_sfm
${STLPLUS_LIBRARY})
SET_PROPERTY(TARGET ${ProjectName} PROPERTY FOLDER OpenMVG/software/ui)
INSTALL(TARGETS ${ProjectName} DESTINATION bin/)