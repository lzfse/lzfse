# Copyright (c) 2015-2016, Apple Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:  
#
# 1.  Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
# 2.  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#     following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3.  Neither the name of the copyright holder(s) nor the names of any contributors may be used to endorse or
#     promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

project(lzfse C)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -fvisibility=hidden")
add_compile_options(-Wall -Wno-unknown-pragmas -Wno-unused-variable)
add_definitions(-D_POSIX_C_SOURCE)

add_library(lzfse STATIC
            src/lzfse_decode.c
            src/lzfse_decode_base.c
            src/lzfse_encode.c
            src/lzfse_encode_base.c
            src/lzfse_fse.c
            src/lzvn_decode_base.c
            src/lzvn_encode_base.c)
install(TARGETS lzfse DESTINATION lib)
install(FILES src/lzfse.h DESTINATION include)

add_executable(lzfse_cmd
               src/lzfse_main.c)
target_link_libraries(lzfse_cmd lzfse)
set_target_properties(lzfse_cmd PROPERTIES OUTPUT_NAME lzfse)
install(TARGETS lzfse_cmd DESTINATION bin)
