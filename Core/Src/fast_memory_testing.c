/*
 * fast_memory_testing.c
 *
 *  Created on: Aug 15, 2025
 *      Author: YZhang
 */



#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "fast_memory_testing.h"

/// Framework code

// it is used later when we want to see if the start/end/length matches the MemoryTesting_Word_t.
#define IS_ALIGNED(x, align) (((x) & ((align) - 1)) == 0)       // only works for power of 2

typedef enum RWTYPE_T
{
    READING,
    WRITING
} RWType_t;

typedef MemoryTesting_Word_t (*WordGenerator_Fn_t)(RWType_t rwType, unsigned int round, unsigned int wordIndex, void *context);

static MemoryTesting_Status_t PerformTestOneRound(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int round, WordGenerator_Fn_t wordGeneratorFn, void *wordGeneratorContext)
{
    MemoryTesting_Address_t end = start + (MemoryTesting_Address_t)length;

    // Write test data.
    for (MemoryTesting_Address_t address = start; address < end; address += sizeof(MemoryTesting_Word_t))
    {
        unsigned int wordIndex = (unsigned int)(address - start) / (unsigned int)sizeof(MemoryTesting_Word_t); // assigning 1,2,3,4 to each memory address at test
        MemoryTesting_Word_t data = wordGeneratorFn(WRITING, round, wordIndex, wordGeneratorContext);

        if (!context->writeFn(address, data, context->userData))
            return MEMORY_TESTING_STATUS_IO_ERROR;
    }

    // Commit the written data (if needed).
    if (!context->commitFn(context->userData))                                    // MRAM won't need to commit. Write happens immediately
        return MEMORY_TESTING_STATUS_IO_ERROR;

    // Read back test data.
    for (MemoryTesting_Address_t address = start; address < end; address += sizeof(MemoryTesting_Word_t))
    {
        unsigned int wordIndex = (unsigned int)(address - start) / (unsigned int)sizeof(MemoryTesting_Word_t);
        MemoryTesting_Word_t compareData = wordGeneratorFn(READING, round, wordIndex, wordGeneratorContext);

        MemoryTesting_Word_t data;

        if (!context->readFn(address, &data, context->userData))
            return MEMORY_TESTING_STATUS_IO_ERROR;

        if (data != compareData)
        {
            if (context->errorNotificationFn != NULL)
                context->errorNotificationFn(MEMORY_TESTING_STATUS_CORRUPTION_ERROR, address, context->userData);
            return MEMORY_TESTING_STATUS_CORRUPTION_ERROR;
        }
    }

    return MEMORY_TESTING_STATUS_OK;
}

static MemoryTesting_Status_t PerformTest(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds, WordGenerator_Fn_t wordGeneratorFn, void *wordGeneratorContext)
{
    if (!IS_ALIGNED(start, (MemoryTesting_Address_t)sizeof(MemoryTesting_Word_t)))
        return MEMORY_TESTING_STATUS_INVALID_ARGS;

    if (!IS_ALIGNED(length, (MemoryTesting_Address_t)sizeof(MemoryTesting_Word_t)))
        return MEMORY_TESTING_STATUS_INVALID_ARGS;

    for (unsigned int round = 0; round < rounds; round++)
    {
        MemoryTesting_Status_t error = PerformTestOneRound(context, start, length, round, wordGeneratorFn, wordGeneratorContext);

        if (error != MEMORY_TESTING_STATUS_OK)
            return error;
    }

    return MEMORY_TESTING_STATUS_OK;
}

/// Alternating bits (0xAA)

static MemoryTesting_Word_t AlternatingBits0xAA_WordGenerator(RWType_t rwType, unsigned int round, unsigned int wordIndex, void *v_context)
{
	(void)rwType;
	(void)round;
	(void)wordIndex;
    (void)v_context;

    MemoryTesting_Word_t word;
    memset(&word, 0xAA, sizeof(word)); // NOLINT(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling): memset_s prevents as-if optimization.
    return word;
}

MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_AlternatingBits0xAA(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds)
{
    return PerformTest(context, start, length, rounds, AlternatingBits0xAA_WordGenerator, NULL);
}

/// Alternating bits (0x55)

static MemoryTesting_Word_t AlternatingBits0x55_WordGenerator(RWType_t rwType, unsigned int round, unsigned int wordIndex, void *v_context)
{
	(void)rwType;
	(void)round;
	(void)wordIndex;
    (void)v_context;

    MemoryTesting_Word_t word;
    memset(&word, 0x55, sizeof(word)); // NOLINT(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling): memset_s prevents as-if optimization.
    return word;
}

MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_AlternatingBits0x55(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds)
{
    return PerformTest(context, start, length, rounds, AlternatingBits0x55_WordGenerator, NULL);
}

/// Random values (srand() / rand())

#define SRAND_SEED_1 43
#define SRAND_SEED_2 0xCAFECAFE
#define SRAND_SEED_3 0x20241225

/// @cond
typedef struct RANDWORDGENERATORCONTEXT_T
{
    unsigned int seed;
} RandWordGeneratorContext_t;
/// @endcond

// cppcheck-suppress constParameterCallback
static MemoryTesting_Word_t RandWordGenerator(RWType_t rwType, unsigned int round, unsigned int wordIndex, void *v_context)
{
    (void)rwType;
    (void)round;

    const RandWordGeneratorContext_t *context = (const RandWordGeneratorContext_t *)v_context;

    if (wordIndex == 0)
        srand(context->seed);

    return (MemoryTesting_Word_t)rand();
}

static MemoryTesting_Status_t MemoryTesting_Test_RandPattern(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds, unsigned int seed)
{
    const RandWordGeneratorContext_t wordGeneratorContext =
    {
        .seed = seed
    };

    return PerformTest(context, start, length, rounds, RandWordGenerator, (void *)&wordGeneratorContext);
}

MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_RandPattern1(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds)
{
    return MemoryTesting_Test_RandPattern(context, start, length, rounds, SRAND_SEED_1);
}

MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_RandPattern2(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds)
{
    return MemoryTesting_Test_RandPattern(context, start, length, rounds, SRAND_SEED_2);
}

MEMORY_TESTING_EXPORT MemoryTesting_Status_t MemoryTesting_Test_RandPattern3(const MemoryTesting_Context_t *context, MemoryTesting_Address_t start, size_t length, unsigned int rounds)
{
    return MemoryTesting_Test_RandPattern(context, start, length, rounds, SRAND_SEED_3);
}

