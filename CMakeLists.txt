cmake_minimum_required (VERSION 2.8.11)
project(wireless_charging)

find_package(PkgConfig)


SET(sources 	wireless.c)             
SET(includes  wireless.h)

#add_executable(wireless_charging ${sources} ${includes})
add_library(wlc SHARED ${sources} ${includes})

# tip for install command, define component public headers files
#set_target_properties(wireless_charging PROPERTIES PUBLIC_HEADER  ${includes})


install(TARGETS wlc

        RUNTIME DESTINATION /usr/bin

        LIBRARY DESTINATION /usr/lib

	      PUBLIC_HEADER DESTINATION /usr/include)
