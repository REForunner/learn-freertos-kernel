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
 * A sample implementation of pvPortMalloc() and vPortFree() that permits
 * allocated blocks to be freed, but does not combine adjacent free blocks
 * into a single larger block (and so will fragment memory).  See heap_4.c for
 * an equivalent that does combine adjacent blocks into single larger blocks.
 *
 * See heap_1.c, heap_3.c and heap_4.c for alternative implementations, and the
 * memory management pages of https://www.FreeRTOS.org for more information.
 */
#include <stdlib.h>
#include <string.h>

 /*
特性: 可分配可回收,会导致碎片化     --> 用一个链表实现的,链表上挂着的是空闲的堆区,用到了就拿走,用完了再插进来
                                  --> 这个链表是一个单向链表
                                  --> 各节点按照内存块的大小进行排序

适合用于重复的分配和删除具有相同堆栈空间的任务、队列、信号量、互斥量等等;并且不考虑内存碎片的应用程序
不适用于分配和释放随机字节堆栈空间的应用程序,这将导致碎片化问题尤为突出
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
    #define configHEAP_CLEAR_MEMORY_ON_FREE    0    //当堆被释放后,是否清除内容 0:不清除;1:清除,内容会被写0
#endif

/* A few bytes might be lost to byte aligning the heap start address. */
#define configADJUSTED_HEAP_SIZE    ( configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT )  //同 heap_1 重新调整可分配堆的大小

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE           ( ( size_t ) 8 )    //定义字节的位数

/* Max value that fits in a size_t type. */
#define heapSIZE_MAX                ( ~( ( size_t ) 0 ) )   //定义 size_t 类型的变量的最大值

/* Check if multiplying a and b will result in overflow. */
#define heapMULTIPLY_WILL_OVERFLOW( a, b )    ( ( ( a ) > 0 ) && ( ( b ) > ( heapSIZE_MAX / ( a ) ) ) ) //检查 a * b 是否会溢出  -->  判断结果 0:不会溢出;1:会溢出

/* Check if adding a and b will result in overflow. */
#define heapADD_WILL_OVERFLOW( a, b )         ( ( a ) > ( heapSIZE_MAX - ( b ) ) )      //检查 a + b 是否会溢出 --> 判断结果 0:不会溢出;1:会溢出

/* MSB of the xBlockSize member of an BlockLink_t structure is used to track
 * the allocation status of a block.  When MSB of the xBlockSize member of
 * an BlockLink_t structure is set then the block belongs to the application.
 * When the bit is free the block is still part of the free heap space. */
/*
BlockLink_t结构的xBlockSize成员的MSB用于跟踪块的分配状态。
当设置BlockLink_t结构的xBlockSize成员的MSB时，该块属于应用程序。当位空闲时，块仍然是空闲堆空间的一部分。
*/
#define heapBLOCK_ALLOCATED_BITMASK    ( ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 ) )     //定义 size_t 型变量最高位的掩码值
#define heapBLOCK_SIZE_IS_VALID( xBlockSize )    ( ( ( xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) == 0 )          //判断变量 xBlockSize 大小的有效性 --> 因为表示堆区块大小的变量的最高位用来表示当前堆区是否被分配
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
    PRIVILEGED_DATA static uint8_t ucHeap[configTOTAL_HEAP_SIZE]; //heap_2内存管理策略中的堆本质上是一个“足够大的数组”   同 heap_1
#endif /* configAPPLICATION_ALLOCATED_HEAP */


/* Define the linked list structure.  This is used to link free blocks in order
 * of their size. */
typedef struct A_BLOCK_LINK     //用来管理堆内存块的链表    --> 按照块的大小链接
{
    struct A_BLOCK_LINK * pxNextFreeBlock; /*<< The next free block in the list. */
    size_t xBlockSize;                     /*<< The size of the free block. */          //表示当前堆内存的大小,包含作为标头的这个结构体;其中这个变量的最高位表示这块堆内存是否已经分配
} BlockLink_t;


static const uint16_t heapSTRUCT_SIZE = ((sizeof(BlockLink_t) + (portBYTE_ALIGNMENT - 1)) & ~((size_t)portBYTE_ALIGNMENT_MASK));   //管理堆内存块链表,在进行字节对齐后的大小
                                                                                                                                    //进行字节对齐的原因是:保证第一次分配堆的起始地址是对齐的,之后只要所有的大小分配只要是对齐的,那么就之后的地址就不会失去已经对齐的特性,提高程序的执行效率
#define heapMINIMUM_BLOCK_SIZE    ( ( size_t ) ( heapSTRUCT_SIZE * 2 ) )    //定义 堆内存块大小 的下限  ：两倍的内存管理结构体的实际大小

/* Create a couple of list links to mark the start and end of the list. */
PRIVILEGED_DATA static BlockLink_t xStart, xEnd;                            //分别用来标记链表的开始与结束

/* Keeps track of the number of free bytes remaining, but says nothing about
 * fragmentation. */
PRIVILEGED_DATA static size_t xFreeBytesRemaining = configADJUSTED_HEAP_SIZE;   //用于统计剩余“未分配”的堆大小 --> 链表中的堆区的总大小（碎片的总大小）

/*-----------------------------------------------------------*/

/*
 * Initialises the heap structures before their first use.
 */
static void prvHeapInit(void) PRIVILEGED_FUNCTION;      //堆初始化的函数声明,在第一次进行堆分配时调用
                                                        // prv前缀：表示函数仅在本文件内有效“类比static修饰”

/*-----------------------------------------------------------*/

/* STATIC FUNCTIONS ARE DEFINED AS MACROS TO MINIMIZE THE FUNCTION CALL DEPTH. */

/*
 * Insert a block into the list of free blocks - which is ordered by size of
 * the block.  Small blocks at the start of the list and large blocks at the end
 * of the list.
 */
 //将一个块插入到空闲块列表中 —— 该列表按块的大小排序。小块位于列表的开头，大块位于列表的末尾。
#define prvInsertBlockIntoFreeList( pxBlockToInsert )                                                                               \
    {                                                                                                                               \
        BlockLink_t * pxIterator;                                                                                                   \
        size_t xBlockSize;                                                                                                          \
                                                                                                                                    \
        xBlockSize = pxBlockToInsert->xBlockSize;                                                                                   \
                                                                                                                                    \
        /* Iterate through the list until a block is found that has a larger size */                                                \
        /* than the block we are inserting. */                                                                                      \
        /* 遍历链表,直到找到核是的插入位置 */                                                                                          \
        for (pxIterator = &xStart; pxIterator->pxNextFreeBlock->xBlockSize < xBlockSize; pxIterator = pxIterator->pxNextFreeBlock)  \
        {                                                                                                                           \
            /* There is nothing to do here - just iterate to the correct position. */                                               \
        }                                                                                                                           \
                                                                                                                                    \
        /* Update the list to include the block being inserted in the correct */                                                    \
        /* position. */                                                                                                             \
        /* 在合适的位置插入链表 */                                                                                                    \
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;                                                             \
        pxIterator->pxNextFreeBlock = pxBlockToInsert;                                                                              \
    }
/*-----------------------------------------------------------*/

void * pvPortMalloc( size_t xWantedSize )   //堆的申请      --> “堆大小”分两种概念: 需要的堆大小: 这是申请堆的程序所需要的大小; 实际申请的大小: 这是“需要的堆大小”+描述这个堆区信息的结构体大小+补齐的字节数
{                                           //申请完成后,不初始化堆区
    BlockLink_t* pxBlock;  //指向当前链表
    BlockLink_t* pxPreviousBlock;  //指向前一个链表
    BlockLink_t * pxNewBlockLink;
    PRIVILEGED_DATA static BaseType_t xHeapHasBeenInitialised = pdFALSE;    //堆（链表）的初始化标记
    void * pvReturn = NULL;
    size_t xAdditionalRequiredSize; //记录需要额外申请的大小    链表头 + 对申请字节大小的补齐 = 额外的大小

    vTaskSuspendAll();  //挂起调度器,保护临界资源 --> 以下是原子操作
    {
        /* If this is the first call to malloc then the heap will require
         * initialisation to setup the list of free blocks. */
        if( xHeapHasBeenInitialised == pdFALSE )
        {
            prvHeapInit();  //初次进行堆分配,调用初始化函数并做标记
            xHeapHasBeenInitialised = pdTRUE;
        }

        if( xWantedSize > 0 )
        {
            /* The wanted size must be increased so it can contain a BlockLink_t
             * structure in addition to the requested amount of bytes. Some
             * additional increment may also be needed for alignment. */                                                //链表头用来管理被分配的堆的属性
            xAdditionalRequiredSize = heapSTRUCT_SIZE + portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ); //链表头 + 对申请字节大小的补齐 = 额外的大小

            if( heapADD_WILL_OVERFLOW( xWantedSize, xAdditionalRequiredSize ) == 0 )    //判断实际申请大小的“值”是否溢出    --> 值溢出: 0xFE + 0x2 = 0x01
            {
                xWantedSize += xAdditionalRequiredSize; //计算出实际要申请的大小,并更新申请的大小
            }
            else
            {
                xWantedSize = 0;    //值溢出了,归零,作无效处理
            }
        }

        /* Check the block size we are trying to allocate is not so large that the
         * top bit is set.  The top bit of the block size member of the BlockLink_t
         * structure is used to determine who owns the block - the application or
         * the kernel, so it must be free. */
        if (heapBLOCK_SIZE_IS_VALID(xWantedSize) != 0)   //检验实际申请大小的有效性 --> 应当 < heapSIZE_MAX/2
        {
            if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) ) //判断所需申请的大小是否合法
            {
                /* Blocks are stored in byte order - traverse the list from the start
                 * (smallest) block until one of adequate size is found. */
                 /* 块按大小顺序存储-从开始(最小)块遍历列表，直到找到一个足够大的块。能容纳申请所需的大小 */
                pxPreviousBlock = &xStart;
                pxBlock = xStart.pxNextFreeBlock;

                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )  //当寻找到第一个大于所需大小,或遍历完全部的链表都没有找到合适的目标时终止
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock; //移动到下一个链表
                }

                /* If we found the end marker then a block of adequate size was not found. */
                /* 如果我们找到了结束标记，那么就没有找到足够大的块 */
                if (pxBlock != &xEnd)   //--> 为什么找到了结束标记,就是没有找到足够大的块,原因与堆的初始化（HeapInit）策略有关
                {
                    /* Return the memory space - jumping over the BlockLink_t structure
                     * at its start. */
                    /* 返回内存空间 —— 跳过BlockLink_t结构的起始部分 */
                    pvReturn = (void*)(((uint8_t*)pxPreviousBlock->pxNextFreeBlock) + heapSTRUCT_SIZE); //指针偏移到堆区 --> 堆区的结构是: 堆区的描述信息（控制块） + 堆区（实际上处理数据用到的区域）

                    /* This block is being returned for use so must be taken out of the
                     * list of free blocks. */
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;    //把当前的链表从“链”中去除  --> 让上一个链表指向下一个链表

                    /* If the block is larger than required it can be split into two. */
                    /* 如果块比需要的大，它可以被分割成两个 */
                    if ((pxBlock->xBlockSize - xWantedSize) > heapMINIMUM_BLOCK_SIZE)   //判断分配后,剩余堆区的是否大于最小堆区（“分家”总要有个限度的！）
                    {
                        /* This block is to be split into two.  Create a new block
                         * following the number of bytes requested. The void cast is
                         * used to prevent byte alignment warnings from the compiler. */
                        /* 这一段将被分成两段。根据请求的字节数创建一个新块。void类型转换用于防止编译器发出字节对齐警告。 */
                        pxNewBlockLink = (void*)(((uint8_t*)pxBlock) + xWantedSize);    //把指针分配到新的堆区的首地址 --> 分配完成后,剩余的堆区的首地址

                        /* Calculate the sizes of two blocks split from the single
                         * block. */
                        /* 计算从单个块分割出来的两个块的大小。 */
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize; //更新剩余的堆区的大小
                        pxBlock->xBlockSize = xWantedSize;  //更新分配的堆区的大小

                        /* Insert the new block into the list of free blocks. */
                        prvInsertBlockIntoFreeList( ( pxNewBlockLink ) );       //将剩余的这个“小”堆,插入到堆链表中的合适位置
                    }

                    xFreeBytesRemaining -= pxBlock->xBlockSize; //更新未分配堆大小的值 --> 减去本次分配出去的值

                    /* The block is being returned - it is allocated and owned
                     * by the application and has no "next" block. */
                    heapALLOCATE_BLOCK(pxBlock);        //更新本次分配的堆控制块的信息 --> 置位已分配标记
                    pxBlock->pxNextFreeBlock = NULL;    //                           --> 堆已分配,那么就不再链表中,它指向的下一个链表必须为空,否则后果不可控
                }
            }
        }

        traceMALLOC( pvReturn, xWantedSize );   //跟踪调试用
    }
    ( void ) xTaskResumeAll();  //恢复调度器的运行

    #if ( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )      //判断申请是否成功
        {
            vApplicationMallocFailedHook(); //没有分配成功,就调用钩子函数,来做特殊处理,钩子函数由app提供
        }
    }
    #endif

    return pvReturn;    //返回申请后堆的指针
}
/*-----------------------------------------------------------*/

void vPortFree( void * pv )     //堆的释放
{
    uint8_t * puc = ( uint8_t * ) pv;   //用来存储要释放的堆的控制块首地址
    BlockLink_t * pxLink;   //用于属性结构体的操作

    if( pv != NULL )
    {
        /* The memory being freed will have an BlockLink_t structure immediately
         * before it. */
        /* 被释放的内存在它之前将有一个BlockLink_t结构 */
        puc -= heapSTRUCT_SIZE;                         // --> 被释放的内存结构：|..BlockLink_t..|..内存块..|
                                                        //                 puc->            pv->
        /* This unexpected casting is to keep some compilers from issuing
         * byte alignment warnings. */
        /* 额外的强制转换是为了防止一些编译器发出字节对齐警告 */
        pxLink = (void*)puc;    //给结构体的操作指针“定向”

        configASSERT( heapBLOCK_IS_ALLOCATED( pxLink ) != 0 );  //断言结构体中的内容会是否合法  -> 检查分配标记
        configASSERT( pxLink->pxNextFreeBlock == NULL );        //  -> 检查链表的指向   //描述被释放内存块的链表指向应当指向“NULL”（即不在任何链表中挂着）

        if( heapBLOCK_IS_ALLOCATED( pxLink ) != 0 ) //判断分配标记  --> 前边已经断言过了,这里放心处理
        {                                                         //    如果是一个有分配标记的,且在链表中挂着,运行到这里绝对是有问题的
            if( pxLink->pxNextFreeBlock == NULL )   //判断链表指向
            {
                /* The block is being returned to the heap - it is no longer
                 * allocated. */
                /* 块被返回到堆-它不再被分配。 */
                heapFREE_BLOCK(pxLink);   //清除堆的分配标记
                #if ( configHEAP_CLEAR_MEMORY_ON_FREE == 1 )    //宏：堆在释放时清除内存功能
                {
                    ( void ) memset( puc + heapSTRUCT_SIZE, 0, pxLink->xBlockSize - heapSTRUCT_SIZE );  //清除内存
                }
                #endif

                vTaskSuspendAll();  //挂起调度器    --> 实际需要不进行上下文切换,freertos选择直接挂起调度器,从根源上解决问题;简单粗暴
                {
                    /* Add this block to the list of free blocks. */
                    /* 将此块添加到空闲块列表中。 */
                    prvInsertBlockIntoFreeList(((BlockLink_t*)pxLink)); //内存块释放的本质就是清除分配标记,把这个块插入链表中 --> 这里进行的是最后一步,插入到链表
                    xFreeBytesRemaining += pxLink->xBlockSize;  //更新未使用的堆大小
                    traceFREE( pv, pxLink->xBlockSize );    //调试跟踪
                }
                ( void ) xTaskResumeAll();  //恢复任务调度
            }
        }
    }
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void ) //获取堆的当前剩余大小
{
    return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )  //初始化最初的控制块, heap_2 没有用,这里的定义是为了堵住编译器的嘴！控制块在第一次分配时初始化
{
    /* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

void * pvPortCalloc( size_t xNum,
                     size_t xSize )     //堆的申请,并在申请成功后,将堆区初始化为 0
{
    void * pv = NULL;

    if( heapMULTIPLY_WILL_OVERFLOW( xNum, xSize ) == 0 )    //检查计算结果的溢出情况
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

static void prvHeapInit( void ) /* PRIVILEGED_FUNCTION */   //堆初始化,在第一次进行堆分配时调用
{
    BlockLink_t * pxFirstFreeBlock; //初始化堆结构以后,第一个“事实上”的空内存块（几乎是整个堆区）
    uint8_t * pucAlignedHeap;   //堆对齐后的首地址

    /* Ensure the heap starts on a correctly aligned boundary. */
    /* 确保堆在正确对齐的边界上开始. */
    pucAlignedHeap = (uint8_t*)(((portPOINTER_SIZE_TYPE)&ucHeap[portBYTE_ALIGNMENT - 1]) & (~((portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK)));

    /* xStart is used to hold a pointer to the first item in the list of free
     * blocks.  The void cast is used to prevent compiler warnings. */
    /* xStart用于保存一个指向空闲块列表中第一项的指针。void类型转换用于防止编译器发出警告。*/
    xStart.pxNextFreeBlock = (void*)pucAlignedHeap;     //设置链表头的属性  --> 指向空闲块
    xStart.xBlockSize = ( size_t ) 0;   //链表头的大小是 0 --> 因为链表头只作为一个链表开始的索引,并不参与堆内存分配操作

    /* xEnd is used to mark the end of the list of free blocks. */
    /* xEnd用于标记空闲块列表的末尾 */
    xEnd.xBlockSize = configADJUSTED_HEAP_SIZE; //设置链表尾的属性  --> 结束标记,不应当参与堆的分配,大小是（对齐后）有效的堆区 [实际上应该只起到记录总大小的作用,应该没别的用处,就是个花瓶！]
    xEnd.pxNextFreeBlock = NULL;    //这里已经是堆的尽头,往后的指向是没有意义的,自然应当指向“NULL”  --> [除非你丫的要创造新天地！233333]

    /* To start with there is a single free block that is sized to take up the
     * entire heap space. */
    /* 一个单独的空闲块，其大小占用整个堆空间 */
    pxFirstFreeBlock = (BlockLink_t*)pucAlignedHeap;    //构建“事实上”的堆区;描述结构体起始地址就是堆的实际起始地址    --> 这是一个有效区,以后所有的堆分配都是在这个基础上“再分割”的
    pxFirstFreeBlock->xBlockSize = configADJUSTED_HEAP_SIZE;    //大小是堆对齐后的大小
    pxFirstFreeBlock->pxNextFreeBlock = &xEnd;      //指向的下一个节点是结尾
}
/*-----------------------------------------------------------*/
