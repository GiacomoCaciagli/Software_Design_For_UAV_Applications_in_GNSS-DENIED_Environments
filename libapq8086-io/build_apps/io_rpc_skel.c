#ifndef _IO_RPC_SKEL_H
#define _IO_RPC_SKEL_H
/*******************************************************************************
 * Copyright 2019 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include "io_rpc.h"
#ifndef _QAIC_ENV_H
#define _QAIC_ENV_H

#ifdef __GNUC__
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#else
#pragma GCC diagnostic ignored "-Wpragmas"
#endif
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

#ifndef _ATTRIBUTE_UNUSED

#ifdef _WIN32
#define _ATTRIBUTE_UNUSED
#else
#define _ATTRIBUTE_UNUSED __attribute__ ((unused))
#endif

#endif // _ATTRIBUTE_UNUSED

#ifndef __QAIC_REMOTE
#define __QAIC_REMOTE(ff) ff
#endif //__QAIC_REMOTE

#ifndef __QAIC_HEADER
#define __QAIC_HEADER(ff) ff
#endif //__QAIC_HEADER

#ifndef __QAIC_HEADER_EXPORT
#define __QAIC_HEADER_EXPORT
#endif // __QAIC_HEADER_EXPORT

#ifndef __QAIC_HEADER_ATTRIBUTE
#define __QAIC_HEADER_ATTRIBUTE
#endif // __QAIC_HEADER_ATTRIBUTE

#ifndef __QAIC_IMPL
#define __QAIC_IMPL(ff) ff
#endif //__QAIC_IMPL

#ifndef __QAIC_IMPL_EXPORT
#define __QAIC_IMPL_EXPORT
#endif // __QAIC_IMPL_EXPORT

#ifndef __QAIC_IMPL_ATTRIBUTE
#define __QAIC_IMPL_ATTRIBUTE
#endif // __QAIC_IMPL_ATTRIBUTE

#ifndef __QAIC_STUB
#define __QAIC_STUB(ff) ff
#endif //__QAIC_STUB

#ifndef __QAIC_STUB_EXPORT
#define __QAIC_STUB_EXPORT
#endif // __QAIC_STUB_EXPORT

#ifndef __QAIC_STUB_ATTRIBUTE
#define __QAIC_STUB_ATTRIBUTE
#endif // __QAIC_STUB_ATTRIBUTE

#ifndef __QAIC_SKEL
#define __QAIC_SKEL(ff) ff
#endif //__QAIC_SKEL__

#ifndef __QAIC_SKEL_EXPORT
#define __QAIC_SKEL_EXPORT
#endif // __QAIC_SKEL_EXPORT

#ifndef __QAIC_SKEL_ATTRIBUTE
#define __QAIC_SKEL_ATTRIBUTE
#endif // __QAIC_SKEL_ATTRIBUTE

#ifdef __QAIC_DEBUG__
   #ifndef __QAIC_DBG_PRINTF__
   #include <stdio.h>
   #define __QAIC_DBG_PRINTF__( ee ) do { printf ee ; } while(0)
   #endif
#else
   #define __QAIC_DBG_PRINTF__( ee ) (void)0
#endif


#define _OFFSET(src, sof)  ((void*)(((char*)(src)) + (sof)))

#define _COPY(dst, dof, src, sof, sz)  \
   do {\
         struct __copy { \
            char ar[sz]; \
         };\
         *(struct __copy*)_OFFSET(dst, dof) = *(struct __copy*)_OFFSET(src, sof);\
   } while (0)

#define _COPYIF(dst, dof, src, sof, sz)  \
   do {\
      if(_OFFSET(dst, dof) != _OFFSET(src, sof)) {\
         _COPY(dst, dof, src, sof, sz); \
      } \
   } while (0)

_ATTRIBUTE_UNUSED
static __inline void _qaic_memmove(void* dst, void* src, int size) {
   int i;
   for(i = 0; i < size; ++i) {
      ((char*)dst)[i] = ((char*)src)[i];
   }
}

#define _MEMMOVEIF(dst, src, sz)  \
   do {\
      if(dst != src) {\
         _qaic_memmove(dst, src, sz);\
      } \
   } while (0)


#define _ASSIGN(dst, src, sof)  \
   do {\
      dst = OFFSET(src, sof); \
   } while (0)

#define _STD_STRLEN_IF(str) (str == 0 ? 0 : strlen(str))

#include "AEEStdErr.h"

#define _TRY(ee, func) \
   do { \
      if (AEE_SUCCESS != ((ee) = func)) {\
         __QAIC_DBG_PRINTF__((__FILE__ ":%d:error:%d:%s\n", __LINE__, (int)(ee),#func));\
         goto ee##bail;\
      } \
   } while (0)

#define _CATCH(exception) exception##bail: if (exception != AEE_SUCCESS)

#define _ASSERT(nErr, ff) _TRY(nErr, 0 == (ff) ? AEE_EBADPARM : AEE_SUCCESS)

#ifdef __QAIC_DEBUG__
#define _ALLOCATE(nErr, pal, size, alignment, pv) _TRY(nErr, _allocator_alloc(pal, __FILE_LINE__, size, alignment, (void**)&pv))
#else
#define _ALLOCATE(nErr, pal, size, alignment, pv) _TRY(nErr, _allocator_alloc(pal, 0, size, alignment, (void**)&pv))
#endif


#endif // _QAIC_ENV_H

#include "remote.h"
#ifndef _ALLOCATOR_H
#define _ALLOCATOR_H

#include <stdlib.h>
#include <stdint.h>

typedef struct _heap _heap;
struct _heap {
   _heap* pPrev;
   const char* loc;
   uint64_t buf;
};

typedef struct _allocator {
   _heap* pheap;
   uint8_t* stack;
   uint8_t* stackEnd;
   int nSize;
} _allocator;

_ATTRIBUTE_UNUSED
static __inline int _heap_alloc(_heap** ppa, const char* loc, int size, void** ppbuf) {
   _heap* pn = 0;
   pn = malloc(size + sizeof(_heap) - sizeof(uint64_t));
   if(pn != 0) {
      pn->pPrev = *ppa;
      pn->loc = loc;
      *ppa = pn;
      *ppbuf = (void*)&(pn->buf);
      return 0;
   } else {
      return -1;
   }
}
#define _ALIGN_SIZE(x, y) (((x) + (y-1)) & ~(y-1))

_ATTRIBUTE_UNUSED
static __inline int _allocator_alloc(_allocator* me,
                                    const char* loc,
                                    int size,
                                    unsigned int al,
                                    void** ppbuf) {
   if(size < 0) {
      return -1;
   } else if (size == 0) {
      *ppbuf = 0;
      return 0;
   }
   if((_ALIGN_SIZE((uintptr_t)me->stackEnd, al) + size) < (uintptr_t)me->stack + me->nSize) {
      *ppbuf = (uint8_t*)_ALIGN_SIZE((uintptr_t)me->stackEnd, al);
      me->stackEnd = (uint8_t*)_ALIGN_SIZE((uintptr_t)me->stackEnd, al) + size;
      return 0;
   } else {
      return _heap_alloc(&me->pheap, loc, size, ppbuf);
   }
}

_ATTRIBUTE_UNUSED
static __inline void _allocator_deinit(_allocator* me) {
   _heap* pa = me->pheap;
   while(pa != 0) {
      _heap* pn = pa;
      const char* loc = pn->loc;
      (void)loc;
      pa = pn->pPrev;
      free(pn);
   }
}

_ATTRIBUTE_UNUSED
static __inline void _allocator_init(_allocator* me, uint8_t* stack, int stackSize) {
   me->stack =  stack;
   me->stackEnd =  stack + stackSize;
   me->nSize = stackSize;
   me->pheap = 0;
}


#endif // _ALLOCATOR_H

#ifndef SLIM_H
#define SLIM_H

#include <stdint.h>

//a C data structure for the idl types that can be used to implement
//static and dynamic language bindings fairly efficiently.
//
//the goal is to have a minimal ROM and RAM footprint and without
//doing too many allocations.  A good way to package these things seemed
//like the module boundary, so all the idls within  one module can share
//all the type references.


#define PARAMETER_IN       0x0
#define PARAMETER_OUT      0x1
#define PARAMETER_INOUT    0x2
#define PARAMETER_ROUT     0x3
#define PARAMETER_INROUT   0x4

//the types that we get from idl
#define TYPE_OBJECT             0x0
#define TYPE_INTERFACE          0x1
#define TYPE_PRIMITIVE          0x2
#define TYPE_ENUM               0x3
#define TYPE_STRING             0x4
#define TYPE_WSTRING            0x5
#define TYPE_STRUCTURE          0x6
#define TYPE_UNION              0x7
#define TYPE_ARRAY              0x8
#define TYPE_SEQUENCE           0x9

//these require the pack/unpack to recurse
//so it's a hint to those languages that can optimize in cases where
//recursion isn't necessary.
#define TYPE_COMPLEX_STRUCTURE  (0x10 | TYPE_STRUCTURE)
#define TYPE_COMPLEX_UNION      (0x10 | TYPE_UNION)
#define TYPE_COMPLEX_ARRAY      (0x10 | TYPE_ARRAY)
#define TYPE_COMPLEX_SEQUENCE   (0x10 | TYPE_SEQUENCE)


typedef struct Type Type;

#define INHERIT_TYPE\
   int32_t nativeSize;                /*in the simple case its the same as wire size and alignment*/\
   union {\
      struct {\
         const uintptr_t         p1;\
         const uintptr_t         p2;\
      } _cast;\
      struct {\
         uint32_t  iid;\
         uint32_t  bNotNil;\
      } object;\
      struct {\
         const Type  *arrayType;\
         int32_t      nItems;\
      } array;\
      struct {\
         const Type *seqType;\
         int32_t      nMaxLen;\
      } seqSimple; \
      struct {\
         uint32_t bFloating;\
         uint32_t bSigned;\
      } prim; \
      const SequenceType* seqComplex;\
      const UnionType  *unionType;\
      const StructType *structType;\
      int32_t         stringMaxLen;\
      uint8_t        bInterfaceNotNil;\
   } param;\
   uint8_t    type;\
   uint8_t    nativeAlignment\

typedef struct UnionType UnionType;
typedef struct StructType StructType;
typedef struct SequenceType SequenceType;
struct Type {
   INHERIT_TYPE;
};

struct SequenceType {
   const Type *         seqType;
   uint32_t               nMaxLen;
   uint32_t               inSize;
   uint32_t               routSizePrimIn;
   uint32_t               routSizePrimROut;
};

//byte offset from the start of the case values for
//this unions case value array.  it MUST be aligned
//at the alignment requrements for the descriptor
//
//if negative it means that the unions cases are
//simple enumerators, so the value read from the descriptor
//can be used directly to find the correct case
typedef union CaseValuePtr CaseValuePtr;
union CaseValuePtr {
   const uint8_t*   value8s;
   const uint16_t*  value16s;
   const uint32_t*  value32s;
   const uint64_t*  value64s;
};

//these are only used in complex cases
//so I pulled them out of the type definition as references to make
//the type smaller
struct UnionType {
   const Type           *descriptor;
   uint32_t               nCases;
   const CaseValuePtr   caseValues;
   const Type * const   *cases;
   int32_t               inSize;
   int32_t               routSizePrimIn;
   int32_t               routSizePrimROut;
   uint8_t                inAlignment;
   uint8_t                routAlignmentPrimIn;
   uint8_t                routAlignmentPrimROut;
   uint8_t                inCaseAlignment;
   uint8_t                routCaseAlignmentPrimIn;
   uint8_t                routCaseAlignmentPrimROut;
   uint8_t                nativeCaseAlignment;
   uint8_t              bDefaultCase;
};

struct StructType {
   uint32_t               nMembers;
   const Type * const   *members;
   int32_t               inSize;
   int32_t               routSizePrimIn;
   int32_t               routSizePrimROut;
   uint8_t                inAlignment;
   uint8_t                routAlignmentPrimIn;
   uint8_t                routAlignmentPrimROut;
};

typedef struct Parameter Parameter;
struct Parameter {
   INHERIT_TYPE;
   uint8_t    mode;
   uint8_t  bNotNil;
};

#define SLIM_IFPTR32(is32,is64) (sizeof(uintptr_t) == 4 ? (is32) : (is64))
#define SLIM_SCALARS_IS_DYNAMIC(u) (((u) & 0x00ffffff) == 0x00ffffff)

typedef struct Method Method;
struct Method {
   uint32_t                    uScalars;            //no method index
   int32_t                     primInSize;
   int32_t                     primROutSize;
   int                         maxArgs;
   int                         numParams;
   const Parameter * const     *params;
   uint8_t                       primInAlignment;
   uint8_t                       primROutAlignment;
};

typedef struct Interface Interface;

struct Interface {
   int                            nMethods;
   const Method  * const          *methodArray;
   int                            nIIds;
   const uint32_t                   *iids;
   const uint16_t*                  methodStringArray;
   const uint16_t*                  methodStrings;
   const char*                    strings;
};


#endif //SLIM_H


#ifndef _IO_RPC_SLIM_H
#define _IO_RPC_SLIM_H
#include "remote.h"
#include <stdint.h>

#ifndef __QAIC_SLIM
#define __QAIC_SLIM(ff) ff
#endif
#ifndef __QAIC_SLIM_EXPORT
#define __QAIC_SLIM_EXPORT
#endif

static const Type types[1];
static const Type types[1] = {{0x1,{{(const uintptr_t)0,(const uintptr_t)0}}, 2,0x1}};
static const Parameter parameters[10] = {{0x4,{{(const uintptr_t)0,(const uintptr_t)1}}, 2,0x4,0,0},{SLIM_IFPTR32(0x8,0x10),{{(const uintptr_t)&(types[0]),(const uintptr_t)0x0}}, 9,SLIM_IFPTR32(0x4,0x8),0,0},{0x4,{{(const uintptr_t)0,(const uintptr_t)1}}, 2,0x4,3,0},{SLIM_IFPTR32(0x8,0x10),{{(const uintptr_t)&(types[0]),(const uintptr_t)0x0}}, 9,SLIM_IFPTR32(0x4,0x8),3,0},{0x8,{{(const uintptr_t)0,(const uintptr_t)1}}, 2,0x8,3,0},{0x1,{{(const uintptr_t)0,(const uintptr_t)0}}, 2,0x1,0,0},{0x2,{{(const uintptr_t)0,(const uintptr_t)0}}, 2,0x2,0,0},{0x1,{{(const uintptr_t)0,(const uintptr_t)0}}, 2,0x1,3,0},{0x2,{{(const uintptr_t)0,(const uintptr_t)0}}, 2,0x2,3,0},{0x4,{{(const uintptr_t)0,(const uintptr_t)0}}, 2,0x4,0,0}};
static const Parameter* const parameterArrays[56] = {(&(parameters[0])),(&(parameters[5])),(&(parameters[5])),(&(parameters[5])),(&(parameters[0])),(&(parameters[2])),(&(parameters[3])),(&(parameters[0])),(&(parameters[0])),(&(parameters[0])),(&(parameters[3])),(&(parameters[2])),(&(parameters[3])),(&(parameters[9])),(&(parameters[9])),(&(parameters[9])),(&(parameters[3])),(&(parameters[9])),(&(parameters[9])),(&(parameters[9])),(&(parameters[1])),(&(parameters[9])),(&(parameters[9])),(&(parameters[9])),(&(parameters[9])),(&(parameters[0])),(&(parameters[5])),(&(parameters[8])),(&(parameters[0])),(&(parameters[0])),(&(parameters[5])),(&(parameters[7])),(&(parameters[0])),(&(parameters[0])),(&(parameters[5])),(&(parameters[3])),(&(parameters[0])),(&(parameters[0])),(&(parameters[5])),(&(parameters[6])),(&(parameters[0])),(&(parameters[0])),(&(parameters[5])),(&(parameters[5])),(&(parameters[0])),(&(parameters[0])),(&(parameters[1])),(&(parameters[3])),(&(parameters[0])),(&(parameters[0])),(&(parameters[1])),(&(parameters[0])),(&(parameters[0])),(&(parameters[1])),(&(parameters[2])),(&(parameters[4]))};
static const Method methods[28] = {{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0x8,0x0,2,2,(&(parameterArrays[7])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0x4,0x0,1,1,(&(parameterArrays[0])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x2,0x1,0x0,0x0),0x8,0x4,4,3,(&(parameterArrays[52])),0x4,0x4},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x2,0x0,0x0),0x8,0x4,5,3,(&(parameterArrays[9])),0x4,0x4},{REMOTE_SCALARS_MAKEX(0,0,0x0,0x1,0x0,0x0),0x0,0x8,1,1,(&(parameterArrays[55])),0x1,0x8},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0xc,0x0,3,3,(&(parameterArrays[7])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x2,0x0,0x0,0x0),0x8,0x0,3,2,(&(parameterArrays[45])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x2,0x0,0x0,0x0),0xc,0x0,4,3,(&(parameterArrays[49])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x2,0x1,0x0,0x0),0xc,0x0,6,3,(&(parameterArrays[45])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x2,0x1,0x0,0x0),0x10,0x0,7,4,(&(parameterArrays[45])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0x6,0x0,3,3,(&(parameterArrays[0])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0xc,0x0,4,4,(&(parameterArrays[41])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0x8,0x0,3,3,(&(parameterArrays[37])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0xc,0x0,4,4,(&(parameterArrays[37])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0xc,0x0,5,3,(&(parameterArrays[33])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0x10,0x0,6,4,(&(parameterArrays[33])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0x5,0x1,3,3,(&(parameterArrays[29])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0xc,0x1,4,4,(&(parameterArrays[29])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0x5,0x2,3,3,(&(parameterArrays[25])),0x4,0x2},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0xc,0x2,4,4,(&(parameterArrays[25])),0x4,0x2},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0x8,0x0,4,2,(&(parameterArrays[9])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x2,0x0,0x0),0x18,0x4,11,9,(&(parameterArrays[0])),0x4,0x4},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0x4,0x4,2,2,(&(parameterArrays[4])),0x4,0x4},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0x4,0x0,1,1,(&(parameterArrays[13])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x0,0x0,0x0),0x10,0x0,4,4,(&(parameterArrays[21])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x2,0x0,0x0,0x0),0x10,0x0,5,4,(&(parameterArrays[17])),0x4,0x0},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x1,0x0,0x0),0x10,0x0,6,4,(&(parameterArrays[13])),0x4,0x1},{REMOTE_SCALARS_MAKEX(0,0,0x1,0x3,0x0,0x0),0xc,0x4,8,4,(&(parameterArrays[9])),0x4,0x4}};
static const Method* const methodArrays[43] = {&(methods[0]),&(methods[0]),&(methods[1]),&(methods[2]),&(methods[3]),&(methods[1]),&(methods[1]),&(methods[4]),&(methods[4]),&(methods[5]),&(methods[0]),&(methods[0]),&(methods[1]),&(methods[6]),&(methods[7]),&(methods[8]),&(methods[9]),&(methods[10]),&(methods[11]),&(methods[12]),&(methods[13]),&(methods[14]),&(methods[15]),&(methods[16]),&(methods[17]),&(methods[18]),&(methods[19]),&(methods[20]),&(methods[21]),&(methods[1]),&(methods[1]),&(methods[22]),&(methods[0]),&(methods[1]),&(methods[1]),&(methods[23]),&(methods[24]),&(methods[25]),&(methods[26]),&(methods[23]),&(methods[5]),&(methods[27]),&(methods[1])};
static const char strings[956] = "register_address_num_bits\0sdsp_time_monotonic_ns\0spi_write_reg_word_cs\0spi_write_reg_byte_cs\0sdsp_time_realtime_ns\0spi_read_reg_word_cs\0spi_read_reg_byte_cs\0gpio_is_initialized\0gather_flir_segment\0spi_write_reg_word\0spi_write_reg_byte\0spi_skip_dummy_bit\0uart_set_baud_rate\0spi_read_imu_fifo\0spi_read_reg_word\0spi_read_reg_byte\0register_address\0i2c_slave_config\0gpio_init_output\0mavparser_close\0gpio_init_input\0spi_read_reg_cs\0spi_transfer_cs\0mavparser_read\0mavparser_init\0slave_address\0count_address\0bytes_written\0write_buffer\0packets_read\0fifo_address\0spi_read_reg\0spi_transfer\0spi_write_cs\0spi_set_freq\0read_buffer\0count_speed\0min_packets\0packet_size\0timeout_us\0gpio_close\0gpio_write\0gpio_state\0data_speed\0uart_drain\0uart_flush\0bytes_read\0uart_write\0uart_close\0msgs_read\0i2c_close\0i2c_write\0gpio_read\0spi_write\0spi_close\0uart_read\0uart_init\0i2c_read\0bit_rate\0i2c_init\0gpio_pin\0bus_mode\0spi_init\0baudrate\0outdata\0spi_bus\0cs_gpio\0freq_hz\0status\0buflen\0val\0";
static const uint16_t methodStrings[154] = {273,918,486,540,641,629,527,909,617,697,442,918,909,763,938,843,918,327,0,605,783,918,327,0,514,344,918,472,852,653,115,918,336,909,922,136,918,336,909,922,410,918,336,909,922,49,918,336,952,922,71,918,336,952,922,426,918,680,452,922,457,918,897,945,291,918,336,909,309,918,336,909,553,918,336,909,197,918,336,952,216,918,336,952,566,918,680,452,579,918,909,922,888,918,879,930,823,918,909,730,741,918,909,500,675,870,686,793,870,686,177,914,906,803,918,909,235,918,952,592,918,930,254,918,897,833,918,897,378,918,773,918,861,918,157,870,664,870,361,870,394,870,813,918,93,195,26,195,708,918,719,918,752,918};
static const uint16_t methodStringsArrays[43] = {125,122,152,100,96,150,148,146,144,92,119,116,142,113,88,84,55,80,50,76,45,72,40,68,35,64,30,110,0,140,138,107,104,136,134,132,25,20,15,130,60,10,128};
__QAIC_SLIM_EXPORT const Interface __QAIC_SLIM(io_rpc_slim) = {43,&(methodArrays[0]),0,0,&(methodStringsArrays [0]),methodStrings,strings};
#endif //_IO_RPC_SLIM_H
extern int adsp_mmap_fd_getinfo(int, uint32_t *);
#ifdef __cplusplus
extern "C" {
#endif
static __inline int _skel_method(int (*_pfn)(uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _TRY(_nErr, _pfn(*_in1));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_1(int (*_pfn)(uint32_t, char*, uint32_t, uint32_t*, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   char* _rout2[1];
   uint32_t _rout2Len[1];
   uint32_t _rout3[1];
   char* _rout4[1];
   uint32_t _rout4Len[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint32_t* _primROut;
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 3) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 16);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 4);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_rout2Len, 0, _primIn, 8, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 1);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout2Len[0]));
   _rout2[0] = _praROut[0].buf.pv;
   _COPY(_rout4Len, 0, _primIn, 12, 4);
   _ASSERT(_nErr, (int)((_praROut[1].buf.nLen / 1)) >= (int)(_rout4Len[0]));
   _rout4[0] = _praROut[1].buf.pv;
   _TRY(_nErr, _pfn(*_in1, *_rout2, *_rout2Len, _rout3, *_rout4, *_rout4Len));
   _COPY(_primROut, 0, _rout3, 0, 4);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_2(int (*_pfn)(uint32_t, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _in2[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 16);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_in2, 0, _primIn, 8, 4);
   _COPY(_in3, 0, _primIn, 12, 4);
   _TRY(_nErr, _pfn(*_in1, *_in2, *_in3));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_3(int (*_pfn)(uint32_t, uint32_t, uint32_t, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _in2[1];
   uint32_t _in3[1];
   char* _rout4[1];
   uint32_t _rout4Len[1];
   uint32_t* _primIn;
   int _numIn[1];
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 20);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_in2, 0, _primIn, 8, 4);
   _COPY(_in3, 0, _primIn, 12, 4);
   _COPY(_rout4Len, 0, _primIn, 16, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 0);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout4Len[0]));
   _rout4[0] = _praROut[0].buf.pv;
   _TRY(_nErr, _pfn(*_in1, *_in2, *_in3, *_rout4, *_rout4Len));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_4(int (*_pfn)(uint32_t, uint32_t, uint32_t, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _in2[1];
   uint32_t _in3[1];
   char* _in4[1];
   uint32_t _in4Len[1];
   uint32_t* _primIn;
   remote_arg* _praIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((2 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 20);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_in2, 0, _primIn, 8, 4);
   _COPY(_in3, 0, _primIn, 12, 4);
   _COPY(_in4Len, 0, _primIn, 16, 4);
   _praIn = (_pra + 1);
   _ASSERT(_nErr, (int)((_praIn[0].buf.nLen / 1)) >= (int)(_in4Len[0]));
   _in4[0] = _praIn[0].buf.pv;
   _TRY(_nErr, _pfn(*_in1, *_in2, *_in3, *_in4, *_in4Len));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_5(int (*_pfn)(uint32_t, uint32_t, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _in2[1];
   uint32_t _in3[1];
   uint32_t _in4[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 20);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_in2, 0, _primIn, 8, 4);
   _COPY(_in3, 0, _primIn, 12, 4);
   _COPY(_in4, 0, _primIn, 16, 4);
   _TRY(_nErr, _pfn(*_in1, *_in2, *_in3, *_in4));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_6(int (*_pfn)(uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _in2[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_in2, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in1, *_in2));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_7(int (*_pfn)(uint32_t, uint32_t*), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _rout2[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint32_t* _primROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 4);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _TRY(_nErr, _pfn(*_in1, _rout2));
   _COPY(_primROut, 0, _rout2, 0, 4);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_invoke(uint32_t _mid, uint32_t _sc, remote_arg* _pra) {
   switch(_mid)
   {
      case 31:
      return _skel_method_7((void*)__QAIC_IMPL(io_rpc_gpio_read), _sc, _pra);
      case 32:
      return _skel_method_6((void*)__QAIC_IMPL(io_rpc_gpio_write), _sc, _pra);
      case 33:
      return _skel_method((void*)__QAIC_IMPL(io_rpc_gpio_close), _sc, _pra);
      case 34:
      return _skel_method((void*)__QAIC_IMPL(io_rpc_gpio_is_initialized), _sc, _pra);
      case 35:
      return _skel_method((void*)__QAIC_IMPL(io_rpc_i2c_init), _sc, _pra);
      case 36:
      return _skel_method_5((void*)__QAIC_IMPL(io_rpc_i2c_slave_config), _sc, _pra);
      case 37:
      return _skel_method_4((void*)__QAIC_IMPL(io_rpc_i2c_write), _sc, _pra);
      case 38:
      return _skel_method_3((void*)__QAIC_IMPL(io_rpc_i2c_read), _sc, _pra);
      case 39:
      return _skel_method((void*)__QAIC_IMPL(io_rpc_i2c_close), _sc, _pra);
      case 40:
      return _skel_method_2((void*)__QAIC_IMPL(io_rpc_mavparser_init), _sc, _pra);
      case 41:
      return _skel_method_1((void*)__QAIC_IMPL(io_rpc_mavparser_read), _sc, _pra);
      case 42:
      return _skel_method((void*)__QAIC_IMPL(io_rpc_mavparser_close), _sc, _pra);
   }
   return AEE_EUNSUPPORTED;
}
static __inline int _skel_method_8(int (*_pfn)(uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 4);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _TRY(_nErr, _pfn(*_in0));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_9(int (*_pfn)(uint32_t, uint8_t, uint8_t, uint8_t, uint32_t, uint32_t*, char*, uint32_t, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint8_t _in2[1];
   uint8_t _in3[1];
   uint32_t _in4[1];
   uint32_t _rout5[1];
   char* _rout6[1];
   uint32_t _rout6Len[1];
   uint32_t _in7[1];
   uint32_t _in8[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint32_t* _primROut;
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 2) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 24);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 4);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in2, 0, _primIn, 5, 1);
   _COPY(_in3, 0, _primIn, 6, 1);
   _COPY(_in4, 0, _primIn, 8, 4);
   _COPY(_rout6Len, 0, _primIn, 12, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 1);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout6Len[0]));
   _rout6[0] = _praROut[0].buf.pv;
   _COPY(_in7, 0, _primIn, 16, 4);
   _COPY(_in8, 0, _primIn, 20, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in2, *_in3, *_in4, _rout5, *_rout6, *_rout6Len, *_in7, *_in8));
   _COPY(_primROut, 0, _rout5, 0, 4);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_10(int (*_pfn)(uint32_t, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _rout1[1];
   uint32_t _rout1Len[1];
   uint32_t* _primIn;
   int _numIn[1];
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_rout1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 0);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout1Len[0]));
   _rout1[0] = _praROut[0].buf.pv;
   _TRY(_nErr, _pfn(*_in0, *_rout1, *_rout1Len));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_11(int (*_pfn)(uint32_t, uint8_t, uint16_t*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint16_t _rout2[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint16_t* _primROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 2);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in3, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, _rout2, *_in3));
   _COPY(_primROut, 0, _rout2, 0, 2);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_12(int (*_pfn)(uint32_t, uint8_t, uint16_t*), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint16_t _rout2[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint16_t* _primROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 5);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 2);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _TRY(_nErr, _pfn(*_in0, *_in1, _rout2));
   _COPY(_primROut, 0, _rout2, 0, 2);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_13(int (*_pfn)(uint32_t, uint8_t, uint8_t*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint8_t _rout2[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint8_t* _primROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 1);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in3, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, _rout2, *_in3));
   _COPY(_primROut, 0, _rout2, 0, 1);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_14(int (*_pfn)(uint32_t, uint8_t, uint8_t*), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint8_t _rout2[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint8_t* _primROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 5);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 1);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _TRY(_nErr, _pfn(*_in0, *_in1, _rout2));
   _COPY(_primROut, 0, _rout2, 0, 1);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_15(int (*_pfn)(uint32_t, uint8_t, char*, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   char* _rout2[1];
   uint32_t _rout2Len[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _numIn[1];
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 16);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_rout2Len, 0, _primIn, 8, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 0);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout2Len[0]));
   _rout2[0] = _praROut[0].buf.pv;
   _COPY(_in3, 0, _primIn, 12, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_rout2, *_rout2Len, *_in3));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_16(int (*_pfn)(uint32_t, uint8_t, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   char* _rout2[1];
   uint32_t _rout2Len[1];
   uint32_t* _primIn;
   int _numIn[1];
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_rout2Len, 0, _primIn, 8, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 0);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout2Len[0]));
   _rout2[0] = _praROut[0].buf.pv;
   _TRY(_nErr, _pfn(*_in0, *_in1, *_rout2, *_rout2Len));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_17(int (*_pfn)(uint32_t, uint8_t, uint16_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint16_t _in2[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in2, 0, _primIn, 6, 2);
   _COPY(_in3, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in2, *_in3));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_18(int (*_pfn)(uint32_t, uint8_t, uint16_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint16_t _in2[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in2, 0, _primIn, 6, 2);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in2));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_19(int (*_pfn)(uint32_t, uint8_t, uint8_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint8_t _in2[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in2, 0, _primIn, 5, 1);
   _COPY(_in3, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in2, *_in3));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_20(int (*_pfn)(uint32_t, uint8_t, uint8_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint8_t _in1[1];
   uint8_t _in2[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 6);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 1);
   _COPY(_in2, 0, _primIn, 5, 1);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in2));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_21(int (*_pfn)(uint32_t, char*, uint32_t, char*, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _in1[1];
   uint32_t _in1Len[1];
   char* _rout2[1];
   uint32_t _rout2Len[1];
   uint32_t _in3[1];
   uint32_t* _primIn;
   int _numIn[1];
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((2 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 16);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _ASSERT(_nErr, (int)((_praIn[0].buf.nLen / 1)) >= (int)(_in1Len[0]));
   _in1[0] = _praIn[0].buf.pv;
   _COPY(_rout2Len, 0, _primIn, 8, 4);
   _praROut = (_praIn + _numIn[0] + 0);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout2Len[0]));
   _rout2[0] = _praROut[0].buf.pv;
   _COPY(_in3, 0, _primIn, 12, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in1Len, *_rout2, *_rout2Len, *_in3));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_22(int (*_pfn)(uint32_t, char*, uint32_t, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _in1[1];
   uint32_t _in1Len[1];
   char* _rout2[1];
   uint32_t _rout2Len[1];
   uint32_t* _primIn;
   int _numIn[1];
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((2 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _ASSERT(_nErr, (int)((_praIn[0].buf.nLen / 1)) >= (int)(_in1Len[0]));
   _in1[0] = _praIn[0].buf.pv;
   _COPY(_rout2Len, 0, _primIn, 8, 4);
   _praROut = (_praIn + _numIn[0] + 0);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout2Len[0]));
   _rout2[0] = _praROut[0].buf.pv;
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in1Len, *_rout2, *_rout2Len));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_23(int (*_pfn)(uint32_t, char*, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _in1[1];
   uint32_t _in1Len[1];
   uint32_t _in2[1];
   uint32_t* _primIn;
   remote_arg* _praIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((2 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _ASSERT(_nErr, (int)((_praIn[0].buf.nLen / 1)) >= (int)(_in1Len[0]));
   _in1[0] = _praIn[0].buf.pv;
   _COPY(_in2, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in1Len, *_in2));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_24(int (*_pfn)(uint32_t, char*, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _in1[1];
   uint32_t _in1Len[1];
   uint32_t* _primIn;
   remote_arg* _praIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((2 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _ASSERT(_nErr, (int)((_praIn[0].buf.nLen / 1)) >= (int)(_in1Len[0]));
   _in1[0] = _praIn[0].buf.pv;
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in1Len));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_25(int (*_pfn)(uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_26(int (*_pfn)(uint32_t, uint32_t, uint32_t), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   uint32_t _in1[1];
   uint32_t _in2[1];
   uint32_t* _primIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 0) + (0 + 0))) <= _praEnd);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 12);
   _primIn = _pra[0].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1, 0, _primIn, 4, 4);
   _COPY(_in2, 0, _primIn, 8, 4);
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in2));
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_27(int (*_pfn)(uint64_t*), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint64_t _rout0[1];
   uint64_t* _primROut;
   int _numIn[1];
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((0 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 0);
   _ASSERT(_nErr, _pra[(_numIn[0] + 0)].buf.nLen >= 8);
   _primROut = _pra[(_numIn[0] + 0)].buf.pv;
   _TRY(_nErr, _pfn(_rout0));
   _COPY(_primROut, 0, _rout0, 0, 8);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_28(int (*_pfn)(uint32_t, char*, uint32_t, uint32_t*), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _rout1[1];
   uint32_t _rout1Len[1];
   uint32_t _rout2[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint32_t* _primROut;
   remote_arg* _praIn;
   remote_arg* _praROut;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((1 + 2) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 4);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_rout1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _praROut = (_praIn + _numIn[0] + 1);
   _ASSERT(_nErr, (int)((_praROut[0].buf.nLen / 1)) >= (int)(_rout1Len[0]));
   _rout1[0] = _praROut[0].buf.pv;
   _TRY(_nErr, _pfn(*_in0, *_rout1, *_rout1Len, _rout2));
   _COPY(_primROut, 0, _rout2, 0, 4);
   _CATCH(_nErr) {}
   return _nErr;
}
static __inline int _skel_method_29(int (*_pfn)(uint32_t, char*, uint32_t, uint32_t*), uint32_t _sc, remote_arg* _pra) {
   remote_arg* _praEnd;
   uint32_t _in0[1];
   char* _in1[1];
   uint32_t _in1Len[1];
   uint32_t _rout2[1];
   uint32_t* _primIn;
   int _numIn[1];
   uint32_t* _primROut;
   remote_arg* _praIn;
   int _nErr = 0;
   _praEnd = ((_pra + REMOTE_SCALARS_INBUFS(_sc)) + REMOTE_SCALARS_OUTBUFS(_sc) + REMOTE_SCALARS_INHANDLES(_sc) + REMOTE_SCALARS_OUTHANDLES(_sc));
   _ASSERT(_nErr, (_pra + ((2 + 1) + (0 + 0))) <= _praEnd);
   _numIn[0] = (REMOTE_SCALARS_INBUFS(_sc) - 1);
   _ASSERT(_nErr, _pra[0].buf.nLen >= 8);
   _primIn = _pra[0].buf.pv;
   _ASSERT(_nErr, _pra[(_numIn[0] + 1)].buf.nLen >= 4);
   _primROut = _pra[(_numIn[0] + 1)].buf.pv;
   _COPY(_in0, 0, _primIn, 0, 4);
   _COPY(_in1Len, 0, _primIn, 4, 4);
   _praIn = (_pra + 1);
   _ASSERT(_nErr, (int)((_praIn[0].buf.nLen / 1)) >= (int)(_in1Len[0]));
   _in1[0] = _praIn[0].buf.pv;
   _TRY(_nErr, _pfn(*_in0, *_in1, *_in1Len, _rout2));
   _COPY(_primROut, 0, _rout2, 0, 4);
   _CATCH(_nErr) {}
   return _nErr;
}
__QAIC_SKEL_EXPORT int __QAIC_SKEL(io_rpc_skel_invoke)(uint32_t _sc, remote_arg* _pra) __QAIC_SKEL_ATTRIBUTE {
   switch(REMOTE_SCALARS_METHOD(_sc))
   {
      case 0:
      return _skel_method_25((void*)__QAIC_IMPL(io_rpc_uart_init), _sc, _pra);
      case 1:
      return _skel_method_25((void*)__QAIC_IMPL(io_rpc_uart_set_baud_rate), _sc, _pra);
      case 2:
      return _skel_method_8((void*)__QAIC_IMPL(io_rpc_uart_close), _sc, _pra);
      case 3:
      return _skel_method_29((void*)__QAIC_IMPL(io_rpc_uart_write), _sc, _pra);
      case 4:
      return _skel_method_28((void*)__QAIC_IMPL(io_rpc_uart_read), _sc, _pra);
      case 5:
      return _skel_method_8((void*)__QAIC_IMPL(io_rpc_uart_flush), _sc, _pra);
      case 6:
      return _skel_method_8((void*)__QAIC_IMPL(io_rpc_uart_drain), _sc, _pra);
      case 7:
      return _skel_method_27((void*)__QAIC_IMPL(io_rpc_sdsp_time_monotonic_ns), _sc, _pra);
      case 8:
      return _skel_method_27((void*)__QAIC_IMPL(io_rpc_sdsp_time_realtime_ns), _sc, _pra);
      case 9:
      return _skel_method_26((void*)__QAIC_IMPL(io_rpc_spi_init), _sc, _pra);
      case 10:
      return _skel_method_25((void*)__QAIC_IMPL(io_rpc_spi_set_freq), _sc, _pra);
      case 11:
      return _skel_method_25((void*)__QAIC_IMPL(io_rpc_spi_skip_dummy_bit), _sc, _pra);
      case 12:
      return _skel_method_8((void*)__QAIC_IMPL(io_rpc_spi_close), _sc, _pra);
      case 13:
      return _skel_method_24((void*)__QAIC_IMPL(io_rpc_spi_write), _sc, _pra);
      case 14:
      return _skel_method_23((void*)__QAIC_IMPL(io_rpc_spi_write_cs), _sc, _pra);
      case 15:
      return _skel_method_22((void*)__QAIC_IMPL(io_rpc_spi_transfer), _sc, _pra);
      case 16:
      return _skel_method_21((void*)__QAIC_IMPL(io_rpc_spi_transfer_cs), _sc, _pra);
      case 17:
      return _skel_method_20((void*)__QAIC_IMPL(io_rpc_spi_write_reg_byte), _sc, _pra);
      case 18:
      return _skel_method_19((void*)__QAIC_IMPL(io_rpc_spi_write_reg_byte_cs), _sc, _pra);
      case 19:
      return _skel_method_18((void*)__QAIC_IMPL(io_rpc_spi_write_reg_word), _sc, _pra);
      case 20:
      return _skel_method_17((void*)__QAIC_IMPL(io_rpc_spi_write_reg_word_cs), _sc, _pra);
      case 21:
      return _skel_method_16((void*)__QAIC_IMPL(io_rpc_spi_read_reg), _sc, _pra);
      case 22:
      return _skel_method_15((void*)__QAIC_IMPL(io_rpc_spi_read_reg_cs), _sc, _pra);
      case 23:
      return _skel_method_14((void*)__QAIC_IMPL(io_rpc_spi_read_reg_byte), _sc, _pra);
      case 24:
      return _skel_method_13((void*)__QAIC_IMPL(io_rpc_spi_read_reg_byte_cs), _sc, _pra);
      case 25:
      return _skel_method_12((void*)__QAIC_IMPL(io_rpc_spi_read_reg_word), _sc, _pra);
      case 26:
      return _skel_method_11((void*)__QAIC_IMPL(io_rpc_spi_read_reg_word_cs), _sc, _pra);
      case 27:
      return _skel_method_10((void*)__QAIC_IMPL(io_rpc_gather_flir_segment), _sc, _pra);
      case 28:
      return _skel_method_9((void*)__QAIC_IMPL(io_rpc_spi_read_imu_fifo), _sc, _pra);
      case 29:
      return _skel_method_8((void*)__QAIC_IMPL(io_rpc_gpio_init_input), _sc, _pra);
      case 30:
      return _skel_method_8((void*)__QAIC_IMPL(io_rpc_gpio_init_output), _sc, _pra);
      case 31:
      {
         uint32_t* _mid;
         if(REMOTE_SCALARS_INBUFS(_sc) < 1 || _pra[0].buf.nLen < 4) { return AEE_EBADPARM; }
         _mid = (uint32_t*)_pra[0].buf.pv;
         return _skel_invoke(*_mid, _sc, _pra);
      }
   }
   return AEE_EUNSUPPORTED;
}
#ifdef __cplusplus
}
#endif
#endif //_IO_RPC_SKEL_H
