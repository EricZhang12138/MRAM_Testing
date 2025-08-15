/*
 * fast_memory_testing.h
 *
 *  Created on: Aug 15, 2025
 *      Author: YZhang
 */

#ifndef INC_FAST_MEMORY_TESTING_H_
#define INC_FAST_MEMORY_TESTING_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "memory_testing_export.h" // IWYU pragma: export

#if SWIG
#define MEMORY_TESTING_EXPORT
#endif

/**
 * @brief Memory testing word type.
 */
#ifndef MEMORY_TESTING_WORD_T
typedef unsigned int MemoryTesting_Word_t;
#else
typedef MEMORY_TESTING_WORD_T MemoryTesting_Word_t;
#endif

/**
 * @brief Memory testing address type.
 */
#ifndef MEMORY_TESTING_ADDRESS_T
typedef uintptr_t MemoryTesting_Address_t;
#else
typedef MEMORY_TESTING_ADDRESS_T MemoryTesting_Address_t;
#endif

/**
 * @brief Memory testing status codes.
 */
typedef enum MEMORY_TESTING_STATUS_T
{
    MEMORY_TESTING_STATUS_OK,
    MEMORY_TESTING_STATUS_INVALID_ARGS,
    MEMORY_TESTING_STATUS_IO_ERROR,
    MEMORY_TESTING_STATUS_CORRUPTION_ERROR
} MemoryTesting_Status_t;

/**
 * @brief Memory read callback.
 * @param[in] address Memory address to read from.
 * @param[out] value Value read from the specified memory address.
 * @param[in] userData Arbitrary user data.
 * @return @c true if the read was successful, @c false otherwise (I/O error).
 */
typedef bool (*MemoryTesting_Context_Read_Fn_t)(MemoryTesting_Address_t address, MemoryTesting_Word_t *value, void *userData);

/**
 * @brief Memory write callback.
 * @param[in] address Memory address to write to.
 * @param[in] value Value to write to the specified memory address.
 * @param[in] userData Arbitrary user data.
 * @return @c true if the write was successful, @c false otherwise (I/O error).
 * @remark This callback is allowed to buffer the values until a subsequent @c commit() happens.
 */
typedef bool (*MemoryTesting_Context_Write_Fn_t)(MemoryTesting_Address_t address, MemoryTesting_Word_t value, void *userData);

/**
 * @brief Memory commit callback. Called after a block of memory has been written to.
 * @param[in] userData Arbitrary user data.
 * @return @c true if the commit was successful, false otherwise (I/O error).
 * @remark Use this to commit any outstanding write operations to memory.
 */
typedef bool (*MemoryTesting_Context_Commit_Fn_t)(void *userData);

/**
 * @brief Error notification callback which provides more detailed information.
 * @param[in] status A status code representing the type of error.
 * @param[in] address Memory address accessed.
 * @param[in] userData Arbitrary user data.
 */
typedef void (*MemoryTesting_Context_ErrorNotification_Fn_t)(MemoryTesting_Status_t status, MemoryTesting_Address_t address, void *userData);

/**
 * @brief Memory testing context.
 */
typedef struct MEMORY_TESTING_CONTEXT_T
{
    MemoryTesting_Context_Read_Fn_t readFn; ///< Memory read callback.
    MemoryTesting_Context_Write_Fn_t writeFn; ///< Memory write callback.
    MemoryTesting_Context_Commit_Fn_t commitFn; ///< Memory commit callback.
    MemoryTesting_Context_ErrorNotification_Fn_t errorNotificationFn; ///< Error notification callback, can be @c NULL.
    void *userData; ///< Arbitrary user data passed through to callbacks.
} MemoryTesting_Context_t;

/**
 * @brief Test memory using an alternating bit pattern (0xAA)
 * @param[in] context Context for testing.
 * @param[in] start Starting memory address to test from.
 * @param[in] length Length of memory range to test.
 * @param[in] rounds Number of rounds of testing to perform.
 * @return A status value.
 */
MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_AlternatingBits0xAA(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds);

/**
 * @brief Test memory using an alternating bit pattern (0x55)
 * @param[in] context Context for testing.
 * @param[in] start Starting memory address to test from.
 * @param[in] length Length of memory range to test.
 * @param[in] rounds Number of rounds of testing to perform.
 * @return A status value.
 */
MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_AlternatingBits0x55(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds);

/**
 * @brief Test memory using random values (pattern 1 / seed 43).
 * @param[in] context Context for testing.
 * @param[in] start Starting memory address to test from.
 * @param[in] length Length of memory range to test.
 * @param[in] rounds Number of rounds of testing to perform.
 * @return A status value.
 * @remark This uses C stdlib's @c srand() / @c rand() and relies on a deterministic implementation (ie: the same seed value will return the same random value sequence).
 */
MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_RandPattern1(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds);

/**
 * @brief Test memory using random values (pattern 2 / seed 0xCAFECAFE).
 * @param[in] context Context for testing.
 * @param[in] start Starting memory address to test from.
 * @param[in] length Length of memory range to test.
 * @param[in] rounds Number of rounds of testing to perform.
 * @return A status value.
 * @remark This uses C stdlib's @c srand() / @c rand() and relies on a deterministic implementation (ie: the same seed value will return the same random value sequence).
 */
MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_RandPattern2(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds);

/**
 * @brief Test memory using random values (pattern 3 / seed 0x20241225).
 * @param[in] context Context for testing.
 * @param[in] start Starting memory address to test from.
 * @param[in] length Length of memory range to test.
 * @param[in] rounds Number of rounds of testing to perform.
 * @return A status value.
 * @remark This uses C stdlib's @c srand() / @c rand() and relies on a deterministic implementation (ie: the same seed value will return the same random value sequence).
 */
MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_RandPattern3(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds);

#ifdef __cplusplus
}
#endif

#endif /* INC_FAST_MEMORY_TESTING_H_ */
