if(${parking_dependencies_DIR})
    set(PROTOBUF_ROOT ${parking_dependencies_DIR}/Linux-x86_64/thirdparty/protobuf/)
endif()

if (NOT EXISTS ${PROTOBUF_ROOT})
    set(PROTOBUF_ROOT /dependencies/Linux-x86_64/thirdparty/protobuf/)
endif()

message("set protobuf root dir: ${PROTOBUF_ROOT}")
option(BUILD_WITH_PROTOBUF "build with protobuf" ON)
add_definitions(-DBUILD_WITH_PROTOBUF=1)

set(PROTOBUF_INCLUDE_DIR ${PROTOBUF_ROOT}/include)
set(PROTOBUF_LIBRARY_DIR ${PROTOBUF_ROOT}/lib)
file(GLOB PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARY_DIR}/libproto*.so)
file(GLOB PROTOBUF_STATIC_LIBS ${PROTOBUF_LIBRARY_DIR}/libproto*.a)
set(Protobuf_INCLUDE_DIR ${PROTOBUF_INCLUDE_DIR})

# set(PROTOBUF_PROTOC_EXECUTABLE ${PROTOBUF_ROOT}/bin/protoc)
set(PROTOBUF_PROTOC_EXECUTABLE $ENV{X64_PROTOBUF_ROOT}/bin/protoc)
if ($ENV{CROSS_COMPILE})
    set(PROTOBUF_PROTOC_EXECUTABLE $ENV{X64_PROTOBUF_ROOT}/bin/protoc)
endif()

set(PROTOC ${PROTOBUF_PROTOC_EXECUTABLE})

#find_package( Protobuf REQUIRED )

set(Protobuf_LIBRARY ${PROTOBUF_LIBRARY_DIR}/libprotobuf.so)

message (STATUS "PROTOBUF_INCLUDE_DIR: ${PROTOBUF_INCLUDE_DIR}")
message (STATUS "PROTOBUF_LIBRARY_DIR: ${PROTOBUF_LIBRARY_DIR}")
message (STATUS "PROTOBUF_LIBRARIES: ${PROTOBUF_LIBRARIES}")
message(STATUS "protoc : ${PROTOC}")

include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIR})
link_directories(${PROTOBUF_LIBRARY_DIR})

if (${CMAKE_SYSTEM_NAME} STREQUAL "Android")

    if(NOT TARGET RTE::log)
        add_library(RTE::log SHARED IMPORTED)
        set_target_properties(RTE::log
            PROPERTIES IMPORTED_LOCATION
            ${SYS_ROOT}/usr/lib/liblog.so)
    endif()

    add_library(proto::protobuf SHARED IMPORTED)
    set_target_properties(proto::protobuf
        PROPERTIES IMPORTED_LOCATION
        ${PROTOBUF_LIBRARY_DIR}/libprotobuf.a)
    set_property(TARGET proto::protobuf APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES RTE::log)
    #target_link_libraries(proto::protobuf INTERFACE RTE::log)
    set(PROTOBUF_LIBRARIES proto::protobuf)
endif()

####################################################################################################

# copy from FindProtobuf.cmake
function(PROTOBUF_GENERATE_CPP OUTPUT_DIR SRCS HDRS)
    if(NOT ARGN)
        message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
        return()
    endif()
    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
        # Create an include path for each file specified
        foreach(FIL ${ARGN})
            get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
            get_filename_component(ABS_PATH ${ABS_FIL} PATH)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    else()
        set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(DEFINED PROTOBUF_IMPORT_DIRS)
        foreach(DIR ${PROTOBUF_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    if(DEFINED POS_PROTO_IMPORT_DIRS)
        foreach(DIR ${POS_PROTO_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    message(STATUS "_protobuf_include_path ${_protobuf_include_path}")
    set(${SRCS})
    set(${HDRS})
    foreach(FIL ${ARGN})
        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
        get_filename_component(FIL_WE ${FIL} NAME_WE)
        get_filename_component(FIL_DIR ${FIL} DIRECTORY)

        list(APPEND ${SRCS} "${OUTPUT_DIR}/${FIL_WE}.pb.cc")
        list(APPEND ${HDRS} "${OUTPUT_DIR}/${FIL_WE}.pb.h")
        add_custom_command(
                OUTPUT "${OUTPUT_DIR}/${FIL_WE}.pb.cc"
                "${OUTPUT_DIR}/${FIL_WE}.pb.h"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                ARGS --cpp_out  ${OUTPUT_DIR} ${_protobuf_include_path} --proto_path ${FIL_DIR} ${FIL_WE}.proto
                DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
                COMMENT "Running C++ protocol buffer compiler on ${FIL}"
                VERBATIM )
    endforeach()
    set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

function(PROTOBUF_GENERATE_PYTHON OUTPUT_DIR SRCS)
    if(NOT ARGN)
        message(SEND_ERROR "Error: PROTOBUF_GENERATE_PYTHON() called without any proto files")
        return()
    endif()
    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
        # Create an include path for each file specified
        foreach(FIL ${ARGN})
            get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
            get_filename_component(ABS_PATH ${ABS_FIL} PATH)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    else()
        set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(DEFINED PROTOBUF_IMPORT_DIRS)
        foreach(DIR ${PROTOBUF_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    if(DEFINED POS_PROTO_IMPORT_DIRS)
        foreach(DIR ${POS_PROTO_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    set(${SRCS})

    add_custom_command(
        OUTPUT "${OUTPUT_DIR}/__init__.py"
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
        COMMAND echo "#!/usr/bin/env python" > ${OUTPUT_DIR}/__init__.py
        COMMENT "Package for python ${OUTPUT_DIR}/__init__.py"
        VERBATIM 
    )
    list(APPEND ${SRCS} ${OUTPUT_DIR}/__init__.py)

    foreach(FIL ${ARGN})
        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
        get_filename_component(FIL_WE ${FIL} NAME_WE)
        list(APPEND ${SRCS} "${OUTPUT_DIR}/${FIL_WE}_pb2.py")
        add_custom_command(
                OUTPUT "${OUTPUT_DIR}/${FIL_WE}_pb2.py"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE} --python_out ${OUTPUT_DIR} ${_protobuf_include_path} ${ABS_FIL}
                DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE} ${OUTPUT_DIR}/__init__.py
                COMMENT "Running Python protocol buffer compiler on ${FIL}"
                VERBATIM )
    endforeach()

    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
endfunction()
function(PROTOBUF_GENERATE_CPP_PY OUTPUT_DIR SRCS HDRS PY)
    if(NOT ARGN)
        message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP_PY() called without any proto files")
        return()
    endif()
    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
        # Create an include path for each file specified
        foreach(FIL ${ARGN})
            get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
            get_filename_component(ABS_PATH ${ABS_FIL} PATH)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    else()
        set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(DEFINED PROTOBUF_IMPORT_DIRS)
        foreach(DIR ${PROTOBUF_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    if(DEFINED POS_PROTO_IMPORT_DIRS)
        foreach(DIR ${POS_PROTO_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    message(STATUS "_protobuf_include_path ${_protobuf_include_path}")
    set(${SRCS})
    set(${HDRS})
    set(${PY})

    add_custom_command(
        OUTPUT "${OUTPUT_DIR}/__init__.py"
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
        COMMAND echo "#!/usr/bin/env python" > ${OUTPUT_DIR}/__init__.py
        COMMENT "Package for python ${OUTPUT_DIR}/__init__.py"
        VERBATIM 
    )
    list(APPEND ${PY} ${OUTPUT_DIR}/__init__.py)

    foreach(FIL ${ARGN})
        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
        get_filename_component(FIL_WE ${FIL} NAME_WE)
        get_filename_component(FIL_DIR ${FIL} DIRECTORY)

        list(APPEND ${SRCS} "${OUTPUT_DIR}/${FIL_WE}.pb.cc")
        list(APPEND ${HDRS} "${OUTPUT_DIR}/${FIL_WE}.pb.h")
        list(APPEND ${PY} "${OUTPUT_DIR}/${FIL_WE}_pb2.py")
        # message(FATAL_ERROR simon ${_protobuf_include_path})
        add_custom_command(
                OUTPUT "${OUTPUT_DIR}/${FIL_WE}.pb.cc"
                "${OUTPUT_DIR}/${FIL_WE}.pb.h"
                "${OUTPUT_DIR}/${FIL_WE}_pb2.py"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                    ARGS --cpp_out  ${OUTPUT_DIR} ${_protobuf_include_path} --proto_path ${FIL_DIR} ${FIL_WE}.proto
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                    ARGS --python_out  ${OUTPUT_DIR} ${_protobuf_include_path} --proto_path ${FIL_DIR} ${FIL_WE}.proto
                #COMMAND touch ${OUTPUT_DIR}/__init__.py
                DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE} ${OUTPUT_DIR}/__init__.py
                COMMENT "Running C++/python protocol buffer compiler on ${FIL}"
                VERBATIM )

    endforeach()

    #message(FATAL_ERROR "${${PY}}")
    set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)
    set(${PY} ${${PY}} PARENT_SCOPE)
endfunction()

function(PROTOBUF_GENERATE_CPP_DESC OUTPUT_DIR SRCS HDRS DESC)
    if(NOT ARGN)
        message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
        return()
    endif()
    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
        # Create an include path for each file specified
        foreach(FIL ${ARGN})
            get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
            get_filename_component(ABS_PATH ${ABS_FIL} PATH)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    else()
        set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(DEFINED PROTOBUF_IMPORT_DIRS)
        foreach(DIR ${PROTOBUF_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    if(DEFINED POS_PROTO_IMPORT_DIRS)
        foreach(DIR ${POS_PROTO_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    message(STATUS "_protobuf_include_path ${_protobuf_include_path}")
    set(${SRCS})
    set(${HDRS})
    set(${DESC})
    foreach(FIL ${ARGN})
        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
        get_filename_component(FIL_WE ${FIL} NAME_WE)
        get_filename_component(FIL_DIR ${FIL} DIRECTORY)

        list(APPEND ${SRCS} "${OUTPUT_DIR}/${FIL_WE}.pb.cc")
        list(APPEND ${HDRS} "${OUTPUT_DIR}/${FIL_WE}.pb.h")
        list(APPEND ${DESC} "${OUTPUT_DIR}/${FIL_WE}.pb.desc")
        add_custom_command(
                OUTPUT "${OUTPUT_DIR}/${FIL_WE}.pb.cc"
                "${OUTPUT_DIR}/${FIL_WE}.pb.h"
                "${OUTPUT_DIR}/${FIL_WE}.pb.desc"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                ARGS --cpp_out  ${OUTPUT_DIR} ${_protobuf_include_path} --proto_path ${FIL_DIR} ${FIL_WE}.proto --descriptor_set_out=${OUTPUT_DIR}/${FIL_WE}.pb.desc
                DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
                COMMENT "Running C++ protocol buffer compiler on ${FIL}"
                VERBATIM )
    endforeach()
    set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)
    set(${DESC} ${${DESC}} PARENT_SCOPE)
endfunction()

function(PROTOBUF_GENERATE_CPP_PY_DESC OUTPUT_DIR SRCS HDRS PY DESC)
    if(NOT ARGN)
        message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP_PY_DESC() called without any proto files")
        return()
    endif()
    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
        # Create an include path for each file specified
        foreach(FIL ${ARGN})
            get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
            get_filename_component(ABS_PATH ${ABS_FIL} PATH)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    else()
        set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(DEFINED PROTOBUF_IMPORT_DIRS)
        foreach(DIR ${PROTOBUF_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    if(DEFINED POS_PROTO_IMPORT_DIRS)
        foreach(DIR ${POS_PROTO_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    message(STATUS "_protobuf_include_path ${_protobuf_include_path}")
    set(${SRCS})
    set(${HDRS})
    set(${PY})
    set(${DESC})

    add_custom_command(
        OUTPUT "${OUTPUT_DIR}/__init__.py"
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
        COMMAND echo "#!/usr/bin/env python" > ${OUTPUT_DIR}/__init__.py
        COMMENT "Package for python ${OUTPUT_DIR}/__init__.py"
        VERBATIM 
    )
    list(APPEND ${PY} ${OUTPUT_DIR}/__init__.py)

    foreach(FIL ${ARGN})
        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
        get_filename_component(FIL_WE ${FIL} NAME_WE)
        get_filename_component(FIL_DIR ${FIL} DIRECTORY)

        list(APPEND ${SRCS} "${OUTPUT_DIR}/${FIL_WE}.pb.cc")
        list(APPEND ${HDRS} "${OUTPUT_DIR}/${FIL_WE}.pb.h")
        list(APPEND ${PY} "${OUTPUT_DIR}/${FIL_WE}_pb2.py")
        list(APPEND ${DESC} "${OUTPUT_DIR}/${FIL_WE}.pb.desc")
        # message(FATAL_ERROR simon ${_protobuf_include_path})
        add_custom_command(
                OUTPUT "${OUTPUT_DIR}/${FIL_WE}.pb.cc"
                "${OUTPUT_DIR}/${FIL_WE}.pb.h"
                "${OUTPUT_DIR}/${FIL_WE}_pb2.py"
                "${OUTPUT_DIR}/${FIL_WE}.pb.desc"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                    ARGS --cpp_out  ${OUTPUT_DIR} ${_protobuf_include_path} --proto_path ${FIL_DIR} ${FIL_WE}.proto --descriptor_set_out=${OUTPUT_DIR}/${FIL_WE}.pb.desc
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                    ARGS --python_out  ${OUTPUT_DIR} ${_protobuf_include_path} --proto_path ${FIL_DIR} ${FIL_WE}.proto
                #COMMAND touch ${OUTPUT_DIR}/__init__.py
                DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE} ${OUTPUT_DIR}/__init__.py
                COMMENT "Running C++/python protocol buffer compiler on ${FIL}"
                VERBATIM )

    endforeach()

    #message(FATAL_ERROR "${${PY}}")
    set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)
    set(${PY} ${${PY}} PARENT_SCOPE)
    set(${DESC} ${${DESC}} PARENT_SCOPE)
endfunction()

# By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
    set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
endif()

# place where to generate protobuf sources
set(proto_gen_folder ${CMAKE_CURRENT_BINARY_DIR}/proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/proto)

