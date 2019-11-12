# CMakeLists for libyuv
# Originally created for "roxlu build system" to compile libyuv on windows
# Run with -DTEST=ON to build unit tests

PROJECT ( YUV C CXX )	# "C" is required even for C++ projects
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
OPTION( TEST "Built unit tests" OFF )
message(${CMAKE_CURRENT_LIST_DIR})
SET ( ly_base_dir	${CMAKE_CURRENT_LIST_DIR})
message(${ly_base_dir})
SET ( ly_src_dir	${ly_base_dir}/source )
SET ( ly_inc_dir	${ly_base_dir}/include )
SET ( ly_tst_dir	${ly_base_dir}/unit_test )
SET ( ly_lib_name	yuv )
SET ( ly_lib_static	${ly_lib_name} )
SET ( ly_lib_shared	${ly_lib_name}_shared )

target_sources(${LRS_TARGET}
    PRIVATE
        "${ly_src_dir}/video_common.cc"
        "${ly_src_dir}/scale_win.cc"
        "${ly_src_dir}/scale_neon64.cc"
        "${ly_src_dir}/scale_neon.cc"
        "${ly_src_dir}/scale_msa.cc"
        "${ly_src_dir}/scale_mmi.cc"
        "${ly_src_dir}/scale_gcc.cc"
        "${ly_src_dir}/scale_common.cc"
        "${ly_src_dir}/scale_argb.cc"
        "${ly_src_dir}/scale_any.cc"
        "${ly_src_dir}/scale.cc"
        "${ly_src_dir}/row_win.cc"
        "${ly_src_dir}/row_neon64.cc"
        "${ly_src_dir}/row_neon.cc"
         "${ly_src_dir}/row_msa.cc"
        "${ly_src_dir}/row_mmi.cc"
        "${ly_src_dir}/row_gcc.cc"
        "${ly_src_dir}/row_common.cc"
        "${ly_src_dir}/row_any.cc"
        "${ly_src_dir}/rotate_win.cc"
        "${ly_src_dir}/rotate_neon64.cc"
        "${ly_src_dir}/rotate_neon.cc"
        "${ly_src_dir}/rotate_msa.cc"
        "${ly_src_dir}/rotate_mmi.cc"
        "${ly_src_dir}/rotate_gcc.cc"
        "${ly_src_dir}/rotate_common.cc"
        "${ly_src_dir}/rotate_argb.cc"
        "${ly_src_dir}/rotate_any.cc"
        "${ly_src_dir}/rotate.cc"
        "${ly_src_dir}/planar_functions.cc"
        "${ly_src_dir}/mjpeg_validate.cc"
        "${ly_src_dir}/mjpeg_decoder.cc"
        "${ly_src_dir}/cpu_id.cc"
        "${ly_src_dir}/convert_to_i420.cc"
        "${ly_src_dir}/convert_to_argb.cc"
        "${ly_src_dir}/convert_jpeg.cc"
        "${ly_src_dir}/convert_from_argb.cc"
        "${ly_src_dir}/convert_from.cc"
        "${ly_src_dir}/convert_argb.cc"
        "${ly_src_dir}/convert.cc"
        "${ly_src_dir}/compare_win.cc"
        "${ly_src_dir}/compare_neon64.cc"
        "${ly_src_dir}/compare_neon.cc"
        "${ly_src_dir}/compare_msa.cc"
        "${ly_src_dir}/compare_mmi.cc"
        "${ly_src_dir}/compare_gcc.cc"
        "${ly_src_dir}/compare_common.cc"
        "${ly_src_dir}/compare.cc"

        #"${ly_inc_dir}/libyuv.h"
)
FILE ( GLOB_RECURSE	ly_source_files ${ly_src_dir}/*.cc )
LIST ( SORT			ly_source_files )

FILE ( GLOB_RECURSE	ly_unittest_sources ${ly_tst_dir}/*.cc )
LIST ( SORT			ly_unittest_sources )

INCLUDE_DIRECTORIES( BEFORE ${ly_inc_dir} )

# this creates the static library (.a)
#ADD_LIBRARY				( ${ly_lib_static} STATIC ${ly_source_files} )

# this creates the shared library (.so)
#ADD_LIBRARY				( ${ly_lib_shared} SHARED ${ly_source_files} )
#SET_TARGET_PROPERTIES	( ${ly_lib_shared} PROPERTIES OUTPUT_NAME "${ly_lib_name}" )
#SET_TARGET_PROPERTIES	( ${ly_lib_shared} PROPERTIES PREFIX "lib" )

# this creates the conversion tool
#ADD_EXECUTABLE			( yuvconvert ${ly_base_dir}/util/yuvconvert.cc )
#TARGET_LINK_LIBRARIES	(${LRS_TARGET} ${ly_lib_static} )


