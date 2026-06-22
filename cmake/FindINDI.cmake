include(FindPackageHandleStandardArgs)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(PC_INDI QUIET libindi)
endif()

find_path(INDI_INCLUDE_DIR
    NAMES indifocuser.h
    HINTS ${PC_INDI_INCLUDE_DIRS}
    PATH_SUFFIXES libindi
)

find_library(INDI_DRIVER_LIBRARY
    NAMES indidriver
    HINTS ${PC_INDI_LIBRARY_DIRS}
)

find_package_handle_standard_args(INDI
    REQUIRED_VARS INDI_INCLUDE_DIR INDI_DRIVER_LIBRARY
)

if(INDI_FOUND)
    set(INDI_INCLUDE_DIRS ${INDI_INCLUDE_DIR})
    set(INDI_LIBRARIES ${INDI_DRIVER_LIBRARY})
endif()

mark_as_advanced(INDI_INCLUDE_DIR INDI_DRIVER_LIBRARY)

