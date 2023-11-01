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
 * The simplest possible implementation of pvPortMalloc().  Note that this
 * implementation does NOT allow allocated memory to be freed again.
 *
 * See heap_2.c, heap_3.c and heap_4.c for alternative implementations, and the
 * memory management pages of https://www.FreeRTOS.org for more information.
 */
#include <stdlib.h>

 /*
 特性: 只管分配,不管回收
 适合对内存动态分配操作敏感的任务;因为内存动态分配操作往往是不可预期的,会使程序的可靠性（安全性）下降   --> [诸如可能发生的堆越界操作,任务的响应时间不确定等]
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

/* A few bytes might be lost to byte aligning the heap start address. */
/* 重新调整堆的大小,因为第一次heap分配时,进行了地址对齐操作,那么heap首地址到对齐的地址之间的一段存储空间可能会被舍弃
    被舍弃的空间大小至多为“对齐字节数的最大余数” */
#define configADJUSTED_HEAP_SIZE    ( configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT )  //--> 为什么要减去的值是一个对齐数,而不是和对齐数余数,猜测的原因可能有两个：
                                                                                    //      1.一般定义这个大小的习惯都是对齐的;即使不是对齐的,在减去一个对齐数以后,它的余数特征也不会发生改变
                                                                                    //      2.一般堆的首地址在链接阶段分配,写程序时“被舍弃的字节数”大小未知;就无法确定重新调整后的堆大小的最优值,那就索性给一个“最合适的值”
                                                                                    //          就算可分配大小比实际大小要小,但也不可能小过一个对齐数!

/* Allocate the memory for the heap. */
#if ( configAPPLICATION_ALLOCATED_HEAP == 1 )

/* The application writer has already defined the array used for the RTOS
* heap - probably so it can be placed in a special segment or address. */
    extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
    static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ]; //heap_1内存管理策略中的堆本质上是一个“足够大的数组”
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* Index into the ucHeap array. */
static size_t xNextFreeByte = ( size_t ) 0; //用于记录已分配堆的大小

/*-----------------------------------------------------------*/

void * pvPortMalloc( size_t xWantedSize )   //堆的申请
{
    void * pvReturn = NULL;
    static uint8_t* pucAlignedHeap = NULL; //指向对齐后的内存堆起始位置

    /* Ensure that blocks are always aligned. */
    /* 确保申请的字节数是对齐字节数的备数 */
    #if ( portBYTE_ALIGNMENT != 1 )
    {
        if( xWantedSize & portBYTE_ALIGNMENT_MASK )
        {
            /* Byte alignment required. Check for overflow. */
            /* 字节数对齐,并判断对齐后的值是否发生溢出 */
            if ((xWantedSize + (portBYTE_ALIGNMENT - (xWantedSize & portBYTE_ALIGNMENT_MASK))) > xWantedSize)
            {
                /* 补齐需要的字节数 */
                xWantedSize += (portBYTE_ALIGNMENT - (xWantedSize & portBYTE_ALIGNMENT_MASK));
            }
            else
            {
                /* 对齐后的字节数产生了溢出,此处清零所需的字节数待后续处理 */
                xWantedSize = 0;
            }
        }
    }
    #endif /* if ( portBYTE_ALIGNMENT != 1 ) */

    vTaskSuspendAll();  //挂起调度器,保护任务之间的临界区 --> 属于FreeRtos层面的实现,但不允许在调度器挂起期间调用可能引发任务调度的API
    {
        if( pucAlignedHeap == NULL )
        {
            /* Ensure the heap starts on a correctly aligned boundary. */
            /* 第一次进行堆分配时,确保堆的起始地址是对齐的 */
            /* 堆的首地址,采用向上取整的方式进行地址的对齐,若采用向下取整的方式,有越界的风险 */
            pucAlignedHeap = (uint8_t*)(((portPOINTER_SIZE_TYPE)&ucHeap[portBYTE_ALIGNMENT - 1]) & (~((portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK)));
        }

        /* Check there is enough room left for the allocation and. */
        if( ( xWantedSize > 0 ) &&                                /* valid size */         // --> 申请一个大小 <= 0 的堆是不合理的
            ( ( xNextFreeByte + xWantedSize ) < configADJUSTED_HEAP_SIZE ) &&              // --> 申请的大小和已经分配的大小之和不能超过堆的大小    [检查堆的上边界,防止溢出]
            ((xNextFreeByte + xWantedSize) > xNextFreeByte)) /* Check for overflow. */     // --> 变量溢出检查,已使用的大小和新申请的大小之和,如果小于已使用的大小,变量一定发生了溢出   [检查不同堆之间的边界,防止越界]
        {
            /* Return the next free byte then increment the index past this
             * block. */
            pvReturn = pucAlignedHeap + xNextFreeByte;  //堆的起始地址 + 已分配的大小 = 尚未分配的堆的起始地址
            xNextFreeByte += xWantedSize;               //更新已分配的堆大小
        }

        traceMALLOC( pvReturn, xWantedSize );   //用于跟踪调试的,一般定义为空,用到了需要自己定义
    }
    ( void ) xTaskResumeAll();  //恢复调度器的运行

    #if ( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )  //判断申请是否成功
        {
            vApplicationMallocFailedHook(); //没有分配成功,就调用钩子函数,来做特殊处理,钩子函数由app提供
        }
    }
    #endif

    return pvReturn;    //返回申请后堆的指针
}
/*-----------------------------------------------------------*/

void vPortFree( void * pv ) //堆的释放      heap_1的堆模型是只分配不回收的,因此并不需要提供释放功能
{
    /* Memory cannot be freed using this scheme.  See heap_2.c, heap_3.c and
     * heap_4.c for alternative implementations, and the memory management pages of
     * https://www.FreeRTOS.org for more information. */
    ( void ) pv;

    /* Force an assert as it is invalid to call this function. */
    configASSERT( pv == NULL );
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )  //初始化局部变量 [清除已分配堆大小记录]
{
    /* Only required when static memory is not cleared. */
    xNextFreeByte = ( size_t ) 0;
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void ) //获取当前剩余的堆大小
{
    return( configADJUSTED_HEAP_SIZE - xNextFreeByte );
}
