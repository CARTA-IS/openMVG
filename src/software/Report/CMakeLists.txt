set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
# The name of my project.
SET(ProjectName openMVG_main_report_reconstruction)
PROJECT(${ProjectName})
# All Source Files here
SET(SourceFiles
main.cpp
ReportGenerator.cpp
)
# All non moc header files here
SET(HeaderFiles
../Geodesy/custom_gcp_registration/document.hpp
ReportGenerator.hpp)
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
