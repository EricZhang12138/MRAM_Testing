/*
 * memory_testing_export.h
 *
 *  Created on: Aug 15, 2025
 *      Author: YZhang
 */

#ifndef MEMORY_TESTING_EXPORT_H
#define MEMORY_TESTING_EXPORT_H

#if defined(_WIN32)
    #if defined(MEMORY_TESTING_BUILD_LIBRARY)
        #define MEMORY_TESTING_EXPORT __declspec(dllexport)
    #else
        #define MEMORY_TESTING_EXPORT __declspec(dllimport)
    #endif
#else
    #define MEMORY_TESTING_EXPORT
#endif

#endif
