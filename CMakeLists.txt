# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(nu-max31875 STATIC EXCLUDE_FROM_ALL)

target_include_directories(nu-max31875
    PUBLIC
        .
)

target_sources(nu-max31875
    PRIVATE
        max31875_c.cpp
        max31875_cpp.cpp
)

target_link_libraries(nu-max31875 PUBLIC mbed-core-flags)
