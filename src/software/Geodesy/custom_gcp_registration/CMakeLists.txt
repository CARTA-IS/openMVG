
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
# The name of my project.
SET(ProjectName openMVG_main_custom_gcp_registration)
PROJECT(${ProjectName})
# All Source Files here
SET(SourceFiles
main.cpp
GCPRegister.cpp
GCP.cpp
GCPList.cpp)
# All non moc header files here
SET(HeaderFiles
document.hpp
GCPRegister.hpp
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
SET_PROPERTY(TARGET ${ProjectName} PROPERTY FOLDER OpenMVG/software)
INSTALL(TARGETS ${ProjectName} DESTINATION bin/)