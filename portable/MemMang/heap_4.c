/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of https://www.FreeRTOS.org for more information.
 */
#include <stdlib.h>
#include <string.h>

/*
特性: heap_4 在 heap_2 的基础上,增加的对相邻碎片的合并功能  --> 由于可以进行碎片合并,因此 heap_4 的链表排序是按照内存地址从小到大排列

heap_4 几乎是FreeRTOS提供的内存管理算法的首选,它定义的堆与 heap_1/heap_2 本质上都是在一个很大的数组上进行的行为,只是管理办法不同

heap_4 基本上是在 heap_2 的功能基础上增加了碎片整理的功能
因此受限于这个特性, heap_4 和 heap_2 有以下两个明显的区别:
1.不同于 heap_2 的链表根据各“内存块的大小”从小到大排列  (保证容器“大小”的递增)
    heap_4 采用的是按照“内存块的起始地址”从小到大排列   (保证容器“位置”的递增)
2.当申请堆时
    heap_4采用的是“最先匹配原则”  --> 从头遍历链表,找到第一个 >= 所需大小的内存块进行再分割分配
                                    此时由于链表的排序不是按照块大小排序,因此找到的第一个不一定
                                    是与所需大小最相近的;故不是最优解,而是先到先得!
    heap_2采用的是“最优匹配原则”  --> 从头遍历链表,找到第一个 >= 所需大小的内存块进行再分割分配
                                    由于链表按照块的大小排列,因此找到的这个块大小一定是最接近所
                                    需的大小的,是一个最优的选择!
*/

 /* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
    #error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

#ifndef configHEAP_CLEAR_MEMORY_ON_FREE
    #define configHEAP_CLEAR_MEMORY_ON_FREE    0    //当堆被释放后,是否清除内容 0:不清楚;1:清除,内容会被写0
#endif

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE    ( ( size_t ) ( xHeapStructSize << 1 ) )   //定义最小的块大小  --> 两倍的属性描述结构大小

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE         ( ( size_t ) 8 )  //定义每个字节的位数 8B     --> 难道还有每个字节不是八位的情况???????

/* Max value that fits in a size_t type. */
#define heapSIZE_MAX              ( ~( ( size_t ) 0 ) )     //定义size_t型变量的最大值 

/* Check if multiplying a and b will result in overflow. */
#define heapMULTIPLY_WILL_OVERFLOW( a, b )    ( ( ( a ) > 0 ) && ( ( b ) > ( heapSIZE_MAX / ( a ) ) ) ) //检查两个数的积是否溢出

/* Check if adding a and b will result in overflow. */
#define heapADD_WILL_OVERFLOW( a, b )         ( ( a ) > ( heapSIZE_MAX - ( b ) ) )  //检查两数之和是否溢出

/* MSB of the xBlockSize member of an BlockLink_t structure is used to track
 * the allocation status of a block.  When MSB of the xBlockSize member of
 * an BlockLink_t structure is set then the block belongs to the application.
 * When the bit is free the block is still part of the free heap space. */
/*
BlockLink_t结构的xBlockSize成员的MSB用于跟踪块的分配状态。
当设置BlockLink_t结构的xBlockSize成员的MSB时，该块属于应用程序。当位空闲时，块仍然是空闲堆空间的一部分。
*/
#define heapBLOCK_ALLOCATED_BITMASK    ( ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 ) ) //定义 size_t 型变量最高位的掩码值
#define heapBLOCK_SIZE_IS_VALID( xBlockSize )    ( ( ( xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) == 0 )  //判断变量 xBlockSize 大小的有效性 --> 因为表示堆区块大小的变量的最高位用来表示当前堆区是否被分配
                                                                                                            //                                   因此单次可申请堆大小的最大值是 heapSIZE_MAX/2
#define heapBLOCK_IS_ALLOCATED( pxBlock )        ( ( ( pxBlock->xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) != 0 ) //判断当前堆区是否已经分配 --> 0:表示空闲;1:表示已分配
#define heapALLOCATE_BLOCK( pxBlock )            ( ( pxBlock->xBlockSize ) |= heapBLOCK_ALLOCATED_BITMASK )         //置位堆区块分配标记
#define heapFREE_BLOCK( pxBlock )                ( ( pxBlock->xBlockSize ) &= ~heapBLOCK_ALLOCATED_BITMASK )        //复位堆区块空闲标记

/*-----------------------------------------------------------*/

/* Allocate the memory for the heap. */
#if ( configAPPLICATION_ALLOCATED_HEAP == 1 )

/* The application writer has already defined the array used for the RTOS
* heap - probably so it can be placed in a special segment or address. */
    extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
    PRIVILEGED_DATA static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ]; //heap_4内存管理策略中的堆本质上是一个“足够大的数组”   同 heap_1 heap_2
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* Define the linked list structure.  This is used to link free blocks in order
 * of their memory address. */
typedef struct A_BLOCK_LINK     //用来管理堆内存块的链表    --> 按照块的大小链接
{
    struct A_BLOCK_LINK * pxNextFreeBlock; /*<< The next free block in the list. */
    size_t xBlockSize;                     /*<< The size of the free block. */  //表示当前堆内存的大小,包含作为标头的这个结构体;其中这个变量的最高位表示这块堆内存是否已经分配
} BlockLink_t;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
/* 将一个正在被释放的内存块插入到空闲内存块列表中的正确位置。被释放的块将与它前面的块和/或它后面的块合并，如果内存块彼此相邻。 */
static void prvInsertBlockIntoFreeList(BlockLink_t* pxBlockToInsert) PRIVILEGED_FUNCTION;   //将块插入到空闲列表中 --> 这个函数是heap_2与heap_4的核心区别之一

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
 /* 在第一次调用pvPortMalloc()时自动调用，以设置所需的堆结构。 */
static void prvHeapInit(void) PRIVILEGED_FUNCTION;      //堆结构初始化

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
 * block must by correctly byte aligned. */
/* 放置在每个分配内存块开头的结构的大小必须按正确的字节对齐 */

//管理堆内存块链表,在进行字节对齐后的大小
//进行字节对齐的原因是:保证第一次分配堆的起始地址是对齐的,之后只要所有的大小分配只要是对齐的,那么就之后的地址就不会失去已经对齐的特性,提高程序的执行效率
static const size_t xHeapStructSize = (sizeof(BlockLink_t) + ((size_t)(portBYTE_ALIGNMENT - 1))) & ~((size_t)portBYTE_ALIGNMENT_MASK);

/* Create a couple of list links to mark the start and end of the list. */
PRIVILEGED_DATA static BlockLink_t xStart;          //用于表示链表头
PRIVILEGED_DATA static BlockLink_t * pxEnd = NULL;  //指向链表尾的BlockLink_t   --> 这里与heap_2的处理是不同的,引起了判断是否是第一次分配堆条件的变化

/* Keeps track of the number of calls to allocate and free memory as well as the
 * number of free bytes remaining, but says nothing about fragmentation. */
/* 跟踪调用的数量分配和自由内存以及剩余的自由字节数，但没有说关于碎片（以下变量都不参与碎片化程度的描述） */
PRIVILEGED_DATA static size_t xFreeBytesRemaining = 0U;     //记录剩余空闲内存的总大小（碎片化程度不描述）
PRIVILEGED_DATA static size_t xMinimumEverFreeBytesRemaining = 0U;  //记录内存块历史剩余的最小可用字节数（历史值）
PRIVILEGED_DATA static size_t xNumberOfSuccessfulAllocations = 0;   //记录堆分配的成功次数（累计值）
PRIVILEGED_DATA static size_t xNumberOfSuccessfulFrees = 0;         //记录堆释放的成功次数（累计值）

/*-----------------------------------------------------------*/

void * pvPortMalloc( size_t xWantedSize )   //堆的申请
{
    BlockLink_t * pxBlock;  //指向当前内存块
    BlockLink_t * pxPreviousBlock;  //指向上一个内存块
    BlockLink_t * pxNewBlockLink;   //指向下一个内存块
    void * pvReturn = NULL;         //存返回值
    size_t xAdditionalRequiredSize; //记录需要额外申请的大小    BlockLink_t对齐后的大小 + 对申请字节大小的补齐 = 额外的大小

    vTaskSuspendAll();  //作用是调度程序被挂起,但是中断是允许响应的 --> 防止上下文切换
    {
        /* If this is the first call to malloc then the heap will require
         * initialisation to setup the list of free blocks. */
        if( pxEnd == NULL )     //判断是否为第一次分配  --> heap_2是用一个静态变量做标记,heap_4使用了链表尾部的特性做标记
        {
            prvHeapInit();  //设置所需的堆结构
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();   //覆盖率测试标记    --> 好像是用来做分支测试用的,在没有 else 的 if 语句中使用了,这里是第一次遇见
        }

        if( xWantedSize > 0 )   //判断申请的大小    --> 一般情况下,申请大小为0的堆是没有意义的,除非申请程序脑子有毛病!!!
        {
            /* The wanted size must be increased so it can contain a BlockLink_t
             * structure in addition to the requested amount of bytes. Some
             * additional increment may also be needed for alignment. */
            xAdditionalRequiredSize = xHeapStructSize + portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ); //计算出额外的字节数

            if( heapADD_WILL_OVERFLOW( xWantedSize, xAdditionalRequiredSize ) == 0 )    //检查实际申请大小数是否溢出
            {
                xWantedSize += xAdditionalRequiredSize; //没有溢出的更新申请大小
            }
            else
            {
                xWantedSize = 0;    //有溢出,直接归零,本次申请做无效处理
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* Check the block size we are trying to allocate is not so large that the
         * top bit is set.  The top bit of the block size member of the BlockLink_t
         * structure is used to determine who owns the block - the application or
         * the kernel, so it must be free. */
        if( heapBLOCK_SIZE_IS_VALID( xWantedSize ) != 0 )   //检查申请大小的有效性  --> 最大值需要 < heapSIZE_MAX/2 ;因为最高位被用来表示这个块是否已经被分配
        {
            if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )   //检查申请大小是否在可分配的有效区间内 0 < ... < 剩余可用的大小
            {
                /* Traverse the list from the start (lowest address) block until
                 * one of adequate size is found. */
                pxPreviousBlock = &xStart;
                pxBlock = xStart.pxNextFreeBlock;   //根据链表头,定位第一个节点

                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )  //寻找符合要求的第一个内存块
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock; //偏移到下一个链表
                }

                /* If the end marker was reached then a block of adequate size
                 * was not found. */
                /* 如果我们找到了结束标记，那么就没有找到足够大的块 */
                if (pxBlock != pxEnd)   //链表头和链表尾不参与分配操作,他们只是作为标记用的
                {
                    /* Return the memory space pointed to - jumping over the
                     * BlockLink_t structure at its start. */
                    pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize ); //指针偏移到内存部分,作为返回值

                    /* This block is being returned for use so must be taken out
                     * of the list of free blocks. */
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;    //从链表中剔除已经分配的

                    /* If the block is larger than required it can be split into
                     * two. */
                    if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )    //判断是否满足再分割要求    --> 分割后的空闲大小大于允许的最小值
                    {
                        /* This block is to be split into two.  Create a new
                         * block following the number of bytes requested. The void
                         * cast is used to prevent byte alignment warnings from the
                         * compiler. */
                        pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );    //找到分割后空闲块的首地址
                        configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );   //地址对齐的断言

                        /* Calculate the sizes of two blocks split from the
                         * single block. */
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize; //更新分割后两个块的大小信息
                        pxBlock->xBlockSize = xWantedSize;

                        /* Insert the new block into the list of free blocks. */
                        prvInsertBlockIntoFreeList( pxNewBlockLink );   //把分割后的空闲块插入到链表中  ==> 里边有碎片整理的功能
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    xFreeBytesRemaining -= pxBlock->xBlockSize; //更新堆剩余空闲大小

                    if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )  //更新堆历史最小值记录
                    {
                        xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* The block is being returned - it is allocated and owned
                     * by the application and has no "next" block. */
                    heapALLOCATE_BLOCK( pxBlock );  //置位分配标记
                    pxBlock->pxNextFreeBlock = NULL;    //下一个链表指向NULL
                    xNumberOfSuccessfulAllocations++;   //更新堆分配成功的次数
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        traceMALLOC( pvReturn, xWantedSize );   //调试跟踪用
    }
    ( void ) xTaskResumeAll();  //恢复程序的正常调度 ==> 允许切换上下文

    #if ( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )
        {
            vApplicationMallocFailedHook(); //分配失败,调用钩子函数
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    #endif /* if ( configUSE_MALLOC_FAILED_HOOK == 1 ) */

    configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );  //指针返回值地址对齐的断言
    return pvReturn;
}
/*-----------------------------------------------------------*/

void vPortFree( void * pv )     //堆的释放
{
    uint8_t * puc = ( uint8_t * ) pv;   //用来存储要释放的堆的控制块首地址
    BlockLink_t * pxLink;       //用于属性结构体（BlockLink_t）的操作

    if( pv != NULL )
    {
        /* The memory being freed will have an BlockLink_t structure immediately
         * before it. */
        puc -= xHeapStructSize;

        /* This casting is to keep the compiler from issuing warnings. */
        pxLink = ( void * ) puc;

        configASSERT( heapBLOCK_IS_ALLOCATED( pxLink ) != 0 );
        configASSERT( pxLink->pxNextFreeBlock == NULL );

        if( heapBLOCK_IS_ALLOCATED( pxLink ) != 0 )
        {
            if( pxLink->pxNextFreeBlock == NULL )
            {
                /* The block is being returned to the heap - it is no longer
                 * allocated. */
                heapFREE_BLOCK( pxLink );
                #if ( configHEAP_CLEAR_MEMORY_ON_FREE == 1 )
                {
                    ( void ) memset( puc + xHeapStructSize, 0, pxLink->xBlockSize - xHeapStructSize );
                }
                #endif

                vTaskSuspendAll();
                {
                    /* Add this block to the list of free blocks. */
                    xFreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( pv, pxLink->xBlockSize );
                    prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
                    xNumberOfSuccessfulFrees++;
                }
                ( void ) xTaskResumeAll();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void ) //获取堆的当前剩余大小
{
    return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

/* 获取自FreeRTOS应用程序开始执行以来堆中存在的未分配字节数的最小值。 */
size_t xPortGetMinimumEverFreeHeapSize(void)
{
    return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )  //初始化最初的控制块, heap_4 没有用,这里的定义是为了堵住编译器的嘴！控制块在第一次分配时初始化
{
    /* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

void * pvPortCalloc( size_t xNum,
                     size_t xSize )     //堆的申请,并在申请成功后,将堆区初始化为 0
{
    void * pv = NULL;

    if( heapMULTIPLY_WILL_OVERFLOW( xNum, xSize ) == 0 )     //检查计算结果的溢出情况
    {
        pv = pvPortMalloc( xNum * xSize );  //申请堆

        if( pv != NULL )
        {
            ( void ) memset( pv, 0, xNum * xSize ); //如果申请成功,把堆区初始化为 0
        }
    }

    return pv;  //返回堆指针
}
/*-----------------------------------------------------------*/

static void prvHeapInit( void ) /* PRIVILEGED_FUNCTION */
{
    BlockLink_t * pxFirstFreeBlock;
    uint8_t * pucAlignedHeap;
    portPOINTER_SIZE_TYPE uxAddress;
    size_t xTotalHeapSize = configTOTAL_HEAP_SIZE;

    /* Ensure the heap starts on a correctly aligned boundary. */
    uxAddress = ( portPOINTER_SIZE_TYPE ) ucHeap;

    if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
    {
        uxAddress += ( portBYTE_ALIGNMENT - 1 );
        uxAddress &= ~( ( portPOINTER_SIZE_TYPE ) portBYTE_ALIGNMENT_MASK );
        xTotalHeapSize -= uxAddress - ( portPOINTER_SIZE_TYPE ) ucHeap;
    }

    pucAlignedHeap = ( uint8_t * ) uxAddress;

    /* xStart is used to hold a pointer to the first item in the list of free
     * blocks.  The void cast is used to prevent compiler warnings. */
    xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
    xStart.xBlockSize = ( size_t ) 0;

    /* pxEnd is used to mark the end of the list of free blocks and is inserted
     * at the end of the heap space. */
    uxAddress = ( ( portPOINTER_SIZE_TYPE ) pucAlignedHeap ) + xTotalHeapSize;
    uxAddress -= xHeapStructSize;
    uxAddress &= ~( ( portPOINTER_SIZE_TYPE ) portBYTE_ALIGNMENT_MASK );
    pxEnd = ( BlockLink_t * ) uxAddress;
    pxEnd->xBlockSize = 0;
    pxEnd->pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
     * entire heap space, minus the space taken by pxEnd. */
    pxFirstFreeBlock = ( BlockLink_t * ) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = ( size_t ) ( uxAddress - ( portPOINTER_SIZE_TYPE ) pxFirstFreeBlock );
    pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

    /* Only one block exists - and it covers the entire usable heap space. */
    xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( BlockLink_t * pxBlockToInsert ) /* PRIVILEGED_FUNCTION */
{
    BlockLink_t * pxIterator;
    uint8_t * puc;

    /* Iterate through the list until a block is found that has a higher address
     * than the block being inserted. */
    for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
    {
        /* Nothing to do here, just iterate to the right position. */
    }

    /* Do the block being inserted, and the block it is being inserted after
     * make a contiguous block of memory? */
    puc = ( uint8_t * ) pxIterator;

    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
    {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    /* Do the block being inserted, and the block it is being inserted before
     * make a contiguous block of memory? */
    puc = ( uint8_t * ) pxBlockToInsert;

    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
    {
        if( pxIterator->pxNextFreeBlock != pxEnd )
        {
            /* Form one big block from the two blocks. */
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        }
        else
        {
            pxBlockToInsert->pxNextFreeBlock = pxEnd;
        }
    }
    else
    {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    /* If the block being inserted plugged a gab, so was merged with the block
     * before and the block after, then it's pxNextFreeBlock pointer will have
     * already been set, and should not be set here as that would make it point
     * to itself. */
    if( pxIterator != pxBlockToInsert )
    {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}
/*-----------------------------------------------------------*/

void vPortGetHeapStats( HeapStats_t * pxHeapStats )
{
    BlockLink_t * pxBlock;
    size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY; /* portMAX_DELAY used as a portable way of getting the maximum value. */

    vTaskSuspendAll();
    {
        pxBlock = xStart.pxNextFreeBlock;

        /* pxBlock will be NULL if the heap has not been initialised.  The heap
         * is initialised automatically when the first allocation is made. */
        if( pxBlock != NULL )
        {
            while( pxBlock != pxEnd )
            {
                /* Increment the number of blocks and record the largest block seen
                 * so far. */
                xBlocks++;

                if( pxBlock->xBlockSize > xMaxSize )
                {
                    xMaxSize = pxBlock->xBlockSize;
                }

                if( pxBlock->xBlockSize < xMinSize )
                {
                    xMinSize = pxBlock->xBlockSize;
                }

                /* Move to the next block in the chain until the last block is
                 * reached. */
                pxBlock = pxBlock->pxNextFreeBlock;
            }
        }
    }
    ( void ) xTaskResumeAll();

    pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
    pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
    pxHeapStats->xNumberOfFreeBlocks = xBlocks;

    taskENTER_CRITICAL();
    {
        pxHeapStats->xAvailableHeapSpaceInBytes = xFreeBytesRemaining;
        pxHeapStats->xNumberOfSuccessfulAllocations = xNumberOfSuccessfulAllocations;
        pxHeapStats->xNumberOfSuccessfulFrees = xNumberOfSuccessfulFrees;
        pxHeapStats->xMinimumEverFreeBytesRemaining = xMinimumEverFreeBytesRemaining;
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/
