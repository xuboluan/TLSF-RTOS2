/* 
 * Two Levels Segregate Fit memory allocator (TLSF)
 * Version 2.4.6
 *
 * Written by Miguel Masmano Tello <mimastel@doctor.upv.es>
 *
 * Thanks to Ismael Ripoll for his suggestions and reviews
 *
 * Copyright (C) 2008, 2007, 2006, 2005, 2004
 *
 * This code is released using a dual license strategy: GPL/LGPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of the GNU General Public License Version 2.0
 * Released under the terms of the GNU Lesser General Public License Version 2.1
 *
 */

/*
 * Code contributions:
 *
 * (Jul 28 2007)  Herman ten Brugge <hermantenbrugge@home.nl>:
 *
 * - Add 64 bit support. It now runs on x86_64 and solaris64.
 * - I also tested this on vxworks/32and solaris/32 and i386/32 processors.
 * - Remove assembly code. I could not measure any performance difference 
 *   on my core2 processor. This also makes the code more portable.
 * - Moved defines/typedefs from tlsf.h to tlsf.c
 * - Changed MIN_BLOCK_SIZE to sizeof (free_ptr_t) and BHDR_OVERHEAD to 
 *   (sizeof (bhdr_t) - MIN_BLOCK_SIZE). This does not change the fact 
 *    that the minumum size is still sizeof 
 *   (bhdr_t).
 * - Changed all C++ comment style to C style. (// -> /.* ... *./)
 * - Used ls_bit instead of ffs and ms_bit instead of fls. I did this to 
 *   avoid confusion with the standard ffs function which returns 
 *   different values.
 * - Created set_bit/clear_bit fuctions because they are not present 
 *   on x86_64.
 * - Added locking support + extra file target.h to show how to use it.
 * - Added get_used_size function (REMOVED in 2.4)
 * - Added rtl_realloc and rtl_calloc function
 * - Implemented realloc clever support.
 * - Added some test code in the example directory.
 * - Bug fixed (discovered by the rockbox project: www.rockbox.org).       
 *
 * (Oct 23 2006) Adam Scislowicz: 
 *
 * - Support for ARMv5 implemented
 *
 */

/*
 * 文件:                          tlsf.c    Version 2.4.6
 * 功能：               动态内存分配——TLSF 二级隔离适应算法 ，实现了malloc  realloc calloc free函数  还有：统计功能的函数与debug相关的函数
 * 中文注释日期:              2010年12月15日
 * 中文注释者：               vector
 * 对此分配算法的理解： （以下相对于32位处理的理解）
                        对于一块内存池，通常为定义的全局数组变量，数组的首地址为整个内存池的首地址，
                        内存池的首地址处存放内存池控制块tlsf，其后为占4个字（sizeof (bhdr_t)）的内存池的块首（首内存块，内存池初始化时，size=8字节（2个字）+ 本块USED +前一块USED），
						内存池的最后2个字空间（8个字节）为整个内存池的块尾（末内存块，内存池初始化时，size =0 + 前一内存块free + 本内存块USED），
						供我们真正使用的动态内存空间为首块到末块之间的空间（不包括首块的空间）；而且每个分配出去的内存块都有一个2字（8字节）的块头。
						注意：此算法把所有空闲的相邻内存块合并为一个空闲内存块（物理相邻），因此，内存池中不会出现： 空闲块+空闲块的现象，
						只会是：USED内存块 + free内存块 + USED内存块，或：USE内存块 + USED内存块的情况，这在内存释放的时候，需要理解，
						更多的理解请阅读 网站 http://rtportal.upv.es/rtmalloc/node/7  上的相关文章。
本程序下载地址：        http://rtportal.upv.es/rtmalloc/node/8

阅读中不解之处：         void *malloc_ex(size_t size, void *mem_pool)函数中， 得到的内存块大于所需内存块时，把内存块分裂为两个内存块，没使用的那个内存块也应该
                         存储上一个内存块（prev）的物理地址吧？b2->prev_hdr = b; 
 */

#include "cmsis_os2.h"                               // CMSIS RTOS header file
#include "cmsis_armclang.h"

#include "tlsf.h"
/*#define USE_SBRK        (0) */
/*#define USE_MMAP        (0) */

#ifndef USE_PRINTF
#define USE_PRINTF      (0)
#endif

#include <string.h>

#ifndef TLSF_USE_LOCKS
#define	TLSF_USE_LOCKS 	(1)
#endif

#ifndef TLSF_STATISTIC
#define	TLSF_STATISTIC 	(1)
#endif

#ifndef USE_MMAP
#define	USE_MMAP 	(0)
#endif

#ifndef USE_SBRK
#define	USE_SBRK 	(0)
#endif

//osMutexAttr_t  *DYNMemMutex; 

const osMutexAttr_t TLSF_Mutex_attr = {
  "TLSF_Mutex",                                            //lock name
   osMutexRecursive|osMutexPrioInherit|osMutexRobust,      //同一线程能多次使用 | 提升线程优先级 | 退出线程自动销毁
   NULL,
   0U    
};


//osMutexId DYNMemMutex_id;
static uint8_t work_mem[DM_MEM_SIZE] = {0};   //Work memory for allocation

static uint8_t mem_errorno = 0;

/*  TLSF上锁解锁函数，可以使用操作系统的内存函数，也可以如下我们自己定义函数*/
#if TLSF_USE_LOCKS       
#include "target.h"
#else
#define TLSF_CREATE_LOCK(_unused_)   do{}while(0)
#define TLSF_DESTROY_LOCK(_unused_)  do{}while(0) 
#define TLSF_ACQUIRE_LOCK(_unused_)  do{}while(0)
#define TLSF_RELEASE_LOCK(_unused_)  do{}while(0)
#endif

/* 统计相关的函数，主要记录使用中的动态内存大小，最大使用量*/
#if TLSF_STATISTIC
#define	TLSF_ADD_SIZE(tlsf, b) do {	/*分配内存块时，增加使用中内存大小*/ \
		tlsf->used_size += (b->size & BLOCK_SIZE) + BHDR_OVERHEAD;	\
		if (tlsf->used_size > tlsf->max_size) 						\
			tlsf->max_size = tlsf->used_size;						\
		} while(0)

#define	TLSF_REMOVE_SIZE(tlsf, b) do {/*释放内存块时，更新used_size*/ \
		tlsf->used_size -= (b->size & BLOCK_SIZE) + BHDR_OVERHEAD;	\
	} while(0)
#else
#define	TLSF_ADD_SIZE(tlsf, b)	     do{}while(0)
#define	TLSF_REMOVE_SIZE(tlsf, b)    do{}while(0)
#endif

#if USE_MMAP || USE_SBRK
#include <unistd.h>
#endif

#if USE_MMAP
#include <sys/mman.h>
#endif

#include "tlsf.h"

#if !defined(__GNUC__)
#ifndef __inline__
#define __inline__
#endif
#endif

/* The  debug functions  only can  be used  when _DEBUG_TLSF_  is set. */
#ifndef _DEBUG_TLSF_
#define _DEBUG_TLSF_  (0)
#endif

/*************************************************************************/
/* Definition of the structures used by TLSF */


/* Some IMPORTANT TLSF parameters */
/* Unlike the preview TLSF versions, now they are statics */
/*内存块理论上的最小值*/
#define BLOCK_ALIGN (sizeof(void *) * 2)  /* 内存块对齐，内存块大小至少是2个字的大小（用于存储struct free_ptr_struct结构体）*/

#define MAX_FLI		(13)                 /*最大内存块的范围2的30次方到2的31次方直接*/
#define MAX_LOG2_SLI	(5)             /*二级数，MAX_SLI表示一级索引分成多少块*/
#define MAX_SLI		(1 << MAX_LOG2_SLI)     /* MAX_SLI = 2^MAX_LOG2_SLI */


#define FLI_OFFSET	(6)     /* tlsf structure just will manage blocks bigger */
/* than 128 bytes */
#define SMALL_BLOCK	(128)
#define REAL_FLI	(MAX_FLI - FLI_OFFSET)  /* 数组最大值*/
#define MIN_BLOCK_SIZE	(sizeof (free_ptr_t))    /*内存块最小值*/
#define BHDR_OVERHEAD	(sizeof (bhdr_t) - MIN_BLOCK_SIZE)  /*内存块的块头的大小*/
#define TLSF_SIGNATURE	(0x2A59FA59)       /*TLSF动态算法的标志*/

#define	PTR_MASK	(sizeof(void *) - 1) 
#define BLOCK_SIZE	(0xFFFFFFFF - PTR_MASK) /* 用于字对齐，处理器取址*/

#define GET_NEXT_BLOCK(_addr, _r) ((bhdr_t *) ((char *) (_addr) + (_r)))  /*得到下一个物理相邻内存块的首地址*/

/*  以下预定义，用于内存的大小，物理地址，内存块最小值等的控制*/
#define	MEM_ALIGN		  ((BLOCK_ALIGN) - 1)
#define ROUNDUP_SIZE(_r)          (((_r) + MEM_ALIGN) & ~MEM_ALIGN) /* _r值低三位 0舎1入（如同四舍五入），并且低三位清零（相对于32位处理器）*/
#define ROUNDDOWN_SIZE(_r)        ((_r) & ~MEM_ALIGN)               /*  _r低三位清零*/
#define ROUNDUP(_x, _v)           ((((~(_x)) + 1) & ((_v)-1)) + (_x)) 

#define BLOCK_STATE	(0x1)   /* 此内存块空闲，前一内存used*/
#define PREV_STATE	(0x2)   /*与上相反*/

/* bit 0 of the block size 表示当前此内存块的状态*/
#define FREE_BLOCK	(0x1)
#define USED_BLOCK	(0x0)

/* bit 1 of the block size */
#define PREV_FREE	(0x2)
#define PREV_USED	(0x0)


#define DEFAULT_AREA_SIZE (1024*10)

#ifdef USE_MMAP
#define PAGE_SIZE (getpagesize())
#endif

/*输出信息*/
#ifdef USE_PRINTF
#include <stdio.h>
# define PRINT_MSG(fmt, args...) printf(fmt, ## args)
# define ERROR_MSG(fmt, args...) printf(fmt, ## args)
#else
# if !defined(PRINT_MSG)
#  define PRINT_MSG(fmt, args...)
# endif
# if !defined(ERROR_MSG)
#  define ERROR_MSG(fmt, args...)
# endif
#endif

typedef unsigned int u32_t;     /* NOTE: Make sure that this type is 4 bytes long on your computer */
typedef unsigned char u8_t;     /* NOTE: Make sure that this type is 1 byte on your computer */

typedef struct free_ptr_struct {
    struct bhdr_struct *prev;
    struct bhdr_struct *next;
} free_ptr_t;

typedef struct bhdr_struct {
    /* This pointer is just valid if the first bit of size is set */
    struct bhdr_struct *prev_hdr;
    /* The size is stored in bytes ，size之后归此bhdr_t控制块管理的内存块大小*/
    size_t size;                /* bit 0 indicates whether the block is used and */
    /* bit 1 allows to know whether the previous block is free */
    union {
        struct free_ptr_struct free_ptr;
        u8_t buffer[1];         /*sizeof(struct free_ptr_struct)]; ，注意读取buffer的值等于读取其地址
								即此联合体的首地址，便于读取ptr的首地址。 */
    } ptr;
} bhdr_t;

/* This structure is embedded at the beginning of each area, giving us
 * enough information to cope with a set of areas */

/*由于连接多个内存区，*/
typedef struct area_info_struct {
    bhdr_t *end;         /*指向末内存块*/
    struct area_info_struct *next;  /*指向下一个内存区，新增的内存*/
} area_info_t;

typedef struct TLSF_struct {
    /* the TLSF's structure signature */
    u32_t tlsf_signature;

#if TLSF_USE_LOCKS
    TLSF_MLOCK_T lock;
#endif

#if TLSF_STATISTIC
    /* These can not be calculated outside tlsf because we
     * do not know the sizes when freeing/reallocing memory. */
    size_t used_size;
    size_t max_size;
#endif

    /* A linked list holding all the existing areas */
    area_info_t *area_head;

    /* the first-level bitmap */
    /* This array should have a size of REAL_FLI bits */
    u32_t fl_bitmap;

    /* the second-level bitmap */
    u32_t sl_bitmap[REAL_FLI];

    bhdr_t *matrix[REAL_FLI][MAX_SLI];
} tlsf_t;


/******************************************************************/
/**************     Helping functions    **************************/
/******************************************************************/
static __inline__ void set_bit(int nr, u32_t * addr);
static __inline__ void clear_bit(int nr, u32_t * addr);
static __inline__ int ls_bit(int x);
static __inline__ int ms_bit(int x);
static __inline__ void MAPPING_SEARCH(size_t * _r, int *_fl, int *_sl);
static __inline__ void MAPPING_INSERT(size_t _r, int *_fl, int *_sl);
static __inline__ bhdr_t *FIND_SUITABLE_BLOCK(tlsf_t * _tlsf, int *_fl, int *_sl);
static __inline__ bhdr_t *process_area(void *area, size_t size);
#if USE_SBRK || USE_MMAP
static __inline__ void *get_new_area(size_t * size);
#endif

/*  数组[256] 索引值（下标）的最高有效值的位置*/
static const int table[] = {
    -1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4,
    4, 4,
    4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5,
    5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6,
    6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6,
    6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7,
    7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7,
    7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7,
    7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7,
    7, 7, 7, 7, 7, 7, 7
};

/*  求数值最低有效位的位置（二进制）*/
static __inline__ int ls_bit(int i)
{
    unsigned int a;
    unsigned int x = i & -i;  /*  x 值 = i只保留最低有效位，其它位清零 */

    a = x <= 0xffff ? (x <= 0xff ? 0 : 8) : (x <= 0xffffff ? 16 : 24);
    return table[x >> a] + a;
}

/*  求数值最高有效位的位置（二进制）*/
static __inline__ int ms_bit(int i)
{
    unsigned int a;
    unsigned int x = (unsigned int) i;

    a = x <= 0xffff ? (x <= 0xff ? 0 : 8) : (x <= 0xffffff ? 16 : 24);
    return table[x >> a] + a;
}

/*  addr[nr >> 5] 中的第nr位置1 （通常nr==[0 31]，则(nr >> 5)==0，如果nr>=32，则二级分割成nr位）  */
static __inline__ void set_bit(int nr, u32_t * addr)
{
    addr[nr >> 5] |= 1 << (nr & 0x1f);
}

/*  addr[nr >> 5] 中的第nr位清0 （通常nr==[0 31]，则(nr >> 5)==0）  */
static __inline__ void clear_bit(int nr, u32_t * addr)
{
    addr[nr >> 5] &= ~(1 << (nr & 0x1f));
}

/*  根据所需内存大小* _r计算出fl（一级）与sl（二级）的值，
注意：从上一内存链表中寻找满足条件的内存块（上一级一定可以满足需要大小的内存），
因此，求得的fl，sl为所需内存的上一级链表的表头索引值。
*/
static __inline__ void MAPPING_SEARCH(size_t * _r, int *_fl, int *_sl)
{
    int _t;

    if (*_r < SMALL_BLOCK) {                          /*  所需内存块小于系统设置的最小内存块时，所需内存块在一级0 */
        *_fl = 0;                                     /*  一级索引为0，二级索引 */
        *_sl = *_r / (SMALL_BLOCK / MAX_SLI);         /*  二级是把128byte等分为MAX_SLI 份*/
    } else {
        _t = (1 << (ms_bit(*_r) - MAX_LOG2_SLI)) - 1; /* _t =  2的ms_bit(*_r)次方/2的ms_bit(*_r)次方-1,
		                                               得到此fl级的二级链表的内存块分割值，即此一级fl中内存块递增值*/
        *_r = *_r + _t;                             /*  需求内存值*_r + 此一级内存块递增值_t = 二级下一个内存链表内存块大小，
		                                              便于求取满足需求内存的索引值*/
        *_fl = ms_bit(*_r);                    /*  得到*_r值的最高有效位（二进制）的位置，即一级索引值fl */
        *_sl = (*_r >> (*_fl - MAX_LOG2_SLI)) - MAX_SLI;   /* 得到*_r所在的二级索引值sl*/
        *_fl -= FLI_OFFSET;                                /*  得到一级索引值fl，即在bitmap中的位置*/
        /*if ((*_fl -= FLI_OFFSET) < 0) // FL wil be always >0!
         *_fl = *_sl = 0;
         */
        *_r &= ~_t; /* 只保留一级与二级索引相关的位，低N位清零*/
    }
}

/* 与以上函数有所不同的是，上面为查找合适内存块，而此为插入内存块
根据_r值得到一级与二级索引值 ， 并没有把内存块_r插入相应空闲链表，与以上函数雷同*/
static __inline__ void MAPPING_INSERT(size_t _r, int *_fl, int *_sl)
{
    if (_r < SMALL_BLOCK) {
        *_fl = 0;
        *_sl = _r / (SMALL_BLOCK / MAX_SLI);
    } else {
        *_fl = ms_bit(_r);
        *_sl = (_r >> (*_fl - MAX_LOG2_SLI)) - MAX_SLI;
        *_fl -= FLI_OFFSET;
    }
}

/*  查找合适内存块的链表表头*/
static __inline__ bhdr_t *FIND_SUITABLE_BLOCK(tlsf_t * _tlsf, int *_fl, int *_sl)
{
    u32_t _tmp = _tlsf->sl_bitmap[*_fl] & (~0 << *_sl);  /*  屏蔽sl_bitmap[*_fl]中的低*_sl位，在此级中寻找空闲块的二级索引*/
    bhdr_t *_b = NULL;

    if (_tmp) {                    /*  此级有空闲内存块 */
        *_sl = ls_bit(_tmp);       /*  得到二级索引值 */
        _b = _tlsf->matrix[*_fl][*_sl];   /*  得到空闲内存块链表的表头*/
    } else {                              /*  如果此一级索引中无空闲内存块，一级的下一索引中查找*/
        *_fl = ls_bit(_tlsf->fl_bitmap & (~0 << (*_fl + 1)));   /*  屏蔽_tlsf->fl_bitmap中的低(*_fl + 1)位，在一级中寻找空闲块的1级索引*/
        if (*_fl > 0) {         /* likely */                  
            *_sl = ls_bit(_tlsf->sl_bitmap[*_fl]);             /*  在*_fl中查找空闲内存二级索引值*_sl */
            _b = _tlsf->matrix[*_fl][*_sl];                    /*  得到空闲内存块链表的表头*/
        }
    }
    return _b;
}

/*  提取表头内存块。更新内存块链表的表头，即把链表头的空闲内存块分配出去后，
    下一个空闲内存块的控制块提上来做表头
    并根据新表头更新位图的标志位，准确表示此链表中有无空闲内存块    
*/
#define EXTRACT_BLOCK_HDR(_b, _tlsf, _fl, _sl) do {					\
		_tlsf -> matrix [_fl] [_sl] = _b -> ptr.free_ptr.next;		\
		if (_tlsf -> matrix[_fl][_sl])	/*新表头非空*/				\
			_tlsf -> matrix[_fl][_sl] -> ptr.free_ptr.prev = NULL;	\
		else { /*新表头为空，即此链表无空闲内存块，更新位图标志位*/		         				\
			clear_bit (_sl, &_tlsf -> sl_bitmap [_fl]);				\
			if (!_tlsf -> sl_bitmap [_fl])							\
				clear_bit (_fl, &_tlsf -> fl_bitmap);				\
		}															\
		_b -> ptr.free_ptr.prev =  NULL;/*清暂时不用的指针，编程的习惯*/	\
		_b -> ptr.free_ptr.next =  NULL;				\
	}while(0)

/*  （删除_b内存块）提取内存块，并根据内存块在链表中的位置调整空闲链表与位图标志位*/
#define EXTRACT_BLOCK(_b, _tlsf, _fl, _sl) do {							\
		if (_b -> ptr.free_ptr.next)/*next非空，连接其后的内存块*/		\
			_b -> ptr.free_ptr.next -> ptr.free_ptr.prev = _b -> ptr.free_ptr.prev; \
		if (_b -> ptr.free_ptr.prev)									\
			_b -> ptr.free_ptr.prev -> ptr.free_ptr.next = _b -> ptr.free_ptr.next; \
		if (_tlsf -> matrix [_fl][_sl] == _b) {	/*此内存块为表头则做如下处理*/   	\
			_tlsf -> matrix [_fl][_sl] = _b -> ptr.free_ptr.next;		\
			if (!_tlsf -> matrix [_fl][_sl]) {	/*更新位图标志位*/						\
				clear_bit (_sl, &_tlsf -> sl_bitmap[_fl]);				\
				if (!_tlsf -> sl_bitmap [_fl])							\
					clear_bit (_fl, &_tlsf -> fl_bitmap);				\
			}															\
		}																\
		_b -> ptr.free_ptr.prev = NULL;					\
		_b -> ptr.free_ptr.next = NULL;					\
	} while(0)

/*  插入内存块，且总是查入表头*/
#define INSERT_BLOCK(_b, _tlsf, _fl, _sl) do {							\
		_b -> ptr.free_ptr.prev = NULL;  /* 插入表头，则前项指针为空*/ \
		_b -> ptr.free_ptr.next = _tlsf -> matrix [_fl][_sl]; \
		if (_tlsf -> matrix [_fl][_sl])	/*若原链表非空，原表头的前项指针指向_b内存块，以形成双向链表*/	\
			_tlsf -> matrix [_fl][_sl] -> ptr.free_ptr.prev = _b;		\
		_tlsf -> matrix [_fl][_sl] = _b;								\
		set_bit (_sl, &_tlsf -> sl_bitmap [_fl]);/*更新位图标志位*/		\
		set_bit (_fl, &_tlsf -> fl_bitmap);								\
	} while(0)



#if USE_SBRK || USE_MMAP
static __inline__ void *get_new_area(size_t * size) 
{
    void *area;

#if USE_SBRK
    area = (void *)sbrk(0);
    if (((void *)sbrk(*size)) != ((void *) -1))
        return area;
#endif

#ifndef MAP_ANONYMOUS
/* https://dev.openwrt.org/ticket/322 */
# define MAP_ANONYMOUS MAP_ANON
#endif


#if USE_MMAP
    *size = ROUNDUP(*size, PAGE_SIZE);
    if ((area = mmap(0, *size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)) != MAP_FAILED)
        return area;
#endif
    return ((void *) ~0);
}
#endif

/*
                    对内存块做相应处理(一般是对新内存池进行初始化)，处理后的结果如下所示:

                      size首地址  --------------------  <------ib  返回值  
*                                   |   *prev_hdr |
                                     -------------
*                                   | size =0x08  |    size的大小为其后联合体的大小| USED_BLOCK | PREV_USED
                                     -------------
*                               /---|    *prev    |
                               /        ————
*                             /     |  *next=NULL |
                     b--------|-> -------------------<--\
*                             |     |   *prev_hdr |      \
                              |      -------------        \
*                             |     |     size    |  此处 | size = 其后内存大小减去一个表头 | USED_BLOCK | PREV_USED;
                              |      -------------        |
*                             |     |  *prev=NULL |       |
                              |        ————           |
*                             |     |  *next=NULL |       |
                              |   -------------------     |
*                             |     |             |       |
                              |      -------------        |
*                             |     |             |       |
                              |        ........           |    
                              |        ........           |
                              |      -------------        |
*                             \     |             |      /
                               \-> ------------------ <-/-----lb
*                                   |   *prev_hdr |——/
                                     -------------
*                                   |     size    |  size =0 | USED_BLOCK | PREV_FREE;
                       size末地址 -------------------
 
  如果调用free释放b块时，不与ib，lb相结合，而是会根据b块的大小把它挂到相应的空闲链表的表头，并更新bitmap位图
*/
static __inline__ bhdr_t *process_area(void *area, size_t size)
{
    bhdr_t *b, *lb, *ib;
    area_info_t *ai;

    ib = (bhdr_t *) area;
    ib->size =
        (sizeof(area_info_t) <
         MIN_BLOCK_SIZE) ? MIN_BLOCK_SIZE : ROUNDUP_SIZE(sizeof(area_info_t)) | USED_BLOCK | PREV_USED;
    b = (bhdr_t *) GET_NEXT_BLOCK(ib->ptr.buffer, ib->size & BLOCK_SIZE);
    b->size = ROUNDDOWN_SIZE(size - 3 * BHDR_OVERHEAD - (ib->size & BLOCK_SIZE)) | USED_BLOCK | PREV_USED;
    b->ptr.free_ptr.prev = b->ptr.free_ptr.next = 0;
    lb = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    lb->prev_hdr = b;
    lb->size = 0 | USED_BLOCK | PREV_FREE;
    ai = (area_info_t *) ib->ptr.buffer;
    ai->next = 0;
    ai->end = lb;
    return ib;
}

/******************************************************************/
/******************** Begin of the allocator code *****************/
/******************************************************************/

char *mp = NULL;         /* 首块内存区的首地址指针 Default memory pool. */
tlsf_t *g_mp = NULL;

/* 初始化内存池，*/

/******************************************************************/
size_t init_memory_pool(size_t mem_pool_size, void *mem_pool)
{
/******************************************************************/
    tlsf_t *tlsf;
    bhdr_t *b, *ib;

	/*  内存池指针非空，内存池大小非零*/
    if (!mem_pool || !mem_pool_size || mem_pool_size < sizeof(tlsf_t) + BHDR_OVERHEAD * 8) {
        ERROR_MSG("init_memory_pool (): memory_pool invalid\n");
        return -1;
    }

    if (((unsigned long) mem_pool & PTR_MASK)) {/*  PTR_MASK =0x03 即mem_pool低两位都为0，对齐到一个字（相对于32位的就是4个字节）*/
        ERROR_MSG("init_memory_pool (): mem_pool must be aligned to a word\n");
        return -1;
    }
    tlsf = (tlsf_t *) mem_pool;   /*此内存区，如果是上电初始化，则内存区为空*/
    /* Check if already initialised 此内存池已经初始化了*/
    if (tlsf->tlsf_signature == TLSF_SIGNATURE) {/* 销毁此内存区（内存池）时，tlsf_signature赋值0*/
        mp = mem_pool;
        b = GET_NEXT_BLOCK(mp, ROUNDUP_SIZE(sizeof(tlsf_t)));
        return b->size & BLOCK_SIZE;
    }

    mp = mem_pool;
		g_mp = mem_pool;

    /* Zeroing the memory pool */
    memset(mem_pool, 0, sizeof(tlsf_t));  /* 内存池首sizeof(tlsf_t)字节清零，*/

    tlsf->tlsf_signature = TLSF_SIGNATURE;

    TLSF_CREATE_LOCK(&tlsf->lock);
    /*  对内存池中tlsf_t控制块之后的内存空间处理，返回bhdr_t类型指针ib*/
    ib = process_area(GET_NEXT_BLOCK
                      (mem_pool, ROUNDUP_SIZE(sizeof(tlsf_t))), ROUNDDOWN_SIZE(mem_pool_size - sizeof(tlsf_t)));
    b = GET_NEXT_BLOCK(ib->ptr.buffer, ib->size & BLOCK_SIZE);  /*  调整指针指向*/
    free_ex(b->ptr.buffer, tlsf); /*  删除b内存块，并根据情况合并内存块，更新相应信息*/
    tlsf->area_head = (area_info_t *) ib->ptr.buffer;  /* tlsf初始化为ib->ptr.buffer*/

#if TLSF_STATISTIC
    tlsf->used_size = mem_pool_size - (b->size & BLOCK_SIZE);
    tlsf->max_size = tlsf->used_size;
#endif

    return (b->size & BLOCK_SIZE);   /* 返回内存池中可用内存大小（总可分配动态内存大小）*/
}

/* 函数功能： 向内存池增加新内存  （此函数算是TLSF中的比较新的功能，也是比较难理解的函数，指针的利用）
   形参    ： area   新内存块地址；   area_size  新内存块大小    mem_pool  原内存池地址指针
   返回    ： size_t：  新增加内存区的大小

           系统初始化后的地址块内存池
  mem_pool首地址-----------------                                                 
*               |   TLSF      |    /----------------------------------------------------------\
                |             |   /                                增加的第二块内存区          \                     增加的第三块内存区
*               | *area_head--|--/                                                              \
  size首地址  --------------------  <---ib           size首地址  -------------------- <---ib     \    size首地址 ------------------  <---ib  
*               |   *prev_hdr |                                     |   *prev_hdr |               \                 |   *prev_hdr |
                 -------------                                       -------------                 \                -------------       
*               | size =0x08  |                                     | size =0x08  |                 \               | size =0x08  |  
                 -------------  <--\                                 ------------- <--\   *area_head \------------>  -------------
*           /---|    *prev    |     \                           /---|    *prev    |    \                        /---|    *prev    |
           /        ————         \--------------\          /        ————        \--------------\       /        ————
*         /     |  *next=NULL |                      \--------/-----|------*next  |                     \-----/-----|----- *next  |    
 b--------|-> -------------------<--\                b--------|-> -------------------<--\            b--------|-> -------------------<--\          
*         |     |   *prev_hdr |      \                        |     |   *prev_hdr |      \                    |     |   *prev_hdr |      \
          |      -------------        \                       |      -------------        \                   |      -------------        \
*         |     |     size    |       |                       |     |     size    |       |                   |     |     size    |       | 
          |      -------------        |                       |      -------------        |                   |      -------------        |
*         |     |  *prev      |       |                       |     |  *prev      |       |                   |     |  *prev      |       |
          |        ————           |                       |        ————           |                   |        ————           |
*         |     |  *next      |       |                       |     |  *next      |       |                   |     |  *next      |       |
          |   -------------------     |                       |   -------------------     |                   |   -------------------     |
*         |     |             |       |                       |     |             |       |                   |     |             |       |
          |      -------------        |                       |      -------------        |                   |      -------------        |
*         |     |             |       |                       |     |             |       |                   |     |             |       |
          |        ........           |                       |        ........           |                   |        ........           |    
          |        ........           |                       |        ........           |                   |        ........           |    
          |      -------------        |                       |      -------------        |                   |      -------------        |
*         \     |             |      /                        \     |             |      /                    \     |             |      /
           \-> ------------------ <-/-----lb                   \-> ------------------ <-/-----lb               \-> ------------------ <-/-----lb
*               |   *prev_hdr |——/                                |   *prev_hdr |——/                            |   *prev_hdr |——/
                 -------------                                       -------------                                   -------------
*               |     size    |                                     |     size    |                                 |     size    |  
   size末地址 -------------------                      size末地址 -------------------                  size末地址 -------------------


                    以上为增加两个新内存区后的情况，*area_head 总是指向链表的首个内存区，而且各个不相邻的物理内存区，通过ib块（内存区的块头）中的next指针连成单向链表。
                    而对于首尾相连的内存区，合并这两个内存区。
*/
/******************************************************************/
size_t add_new_area(void *area, size_t area_size, void *mem_pool)
{
/******************************************************************/
    tlsf_t *tlsf = (tlsf_t *) mem_pool;  /* 原内存池*/
    area_info_t *ptr, *ptr_prev, *ai;
    bhdr_t *ib0, *b0, *lb0, *ib1, *b1, *lb1, *next_b;

    memset(area, 0, area_size);  /* 新增的内存清零*/
    ptr = tlsf->area_head;       /* 得到tlsf->area_head，即第一内存块的块头*/
    ptr_prev = 0;

	/* ib0 bo lb0 表示指向新内存区的指针*/
    ib0 = process_area(area, area_size); /* 对area内存池进行处理*/
    b0 = GET_NEXT_BLOCK(ib0->ptr.buffer, ib0->size & BLOCK_SIZE);
    lb0 = GET_NEXT_BLOCK(b0->ptr.buffer, b0->size & BLOCK_SIZE);/*得到area内存池最后一内存块的块头*/

    /* Before inserting the new area, we have to merge this area with the
       already existing ones */

    while (ptr) {    /* ib1，b1,bl表示原内存池的一些指针*/
        ib1 = (bhdr_t *) ((char *) ptr - BHDR_OVERHEAD);
        b1 = GET_NEXT_BLOCK(ib1->ptr.buffer, ib1->size & BLOCK_SIZE);
        lb1 = ptr->end;

        /* Merging the new area with the next physically contigous one 
		如果新内存区与原内存池的物理地址相连接，并且新内存区在原内存池的前面prev*/
        if ((unsigned long) ib1 == (unsigned long) lb0 + BHDR_OVERHEAD) {
            if (tlsf->area_head == ptr) { /*链表中的首个内存区（TLSF中的area_head指向此区）*/
                tlsf->area_head = ptr->next;
                ptr = ptr->next;
            } else {
                ptr_prev->next = ptr->next;
                ptr = ptr->next;
            }

            b0->size =
                ROUNDDOWN_SIZE((b0->size & BLOCK_SIZE) +
                               (ib1->size & BLOCK_SIZE) + 2 * BHDR_OVERHEAD) | USED_BLOCK | PREV_USED;

            b1->prev_hdr = b0;
            lb0 = lb1;

            continue;
        }

        /* Merging the new area with the previous physically contigousone
		如果新内存区与原内存池的物理地址相连接，并且新内存区在原内存池的后面* */
        if ((unsigned long) lb1->ptr.buffer == (unsigned long) ib0) {
            if (tlsf->area_head == ptr) {
                tlsf->area_head = ptr->next;
                ptr = ptr->next;
            } else {
                ptr_prev->next = ptr->next;
                ptr = ptr->next;
            }

            lb1->size =
                ROUNDDOWN_SIZE((b0->size & BLOCK_SIZE) +
                               (ib0->size & BLOCK_SIZE) + 2 * BHDR_OVERHEAD) | USED_BLOCK | (lb1->size & PREV_STATE);
            next_b = GET_NEXT_BLOCK(lb1->ptr.buffer, lb1->size & BLOCK_SIZE);
            next_b->prev_hdr = lb1;
            b0 = lb1;
            ib0 = ib1;

            continue;
        }
        ptr_prev = ptr;
        ptr = ptr->next;
    }

    /* Inserting the area in the list of linked areas */
    ai = (area_info_t *) ib0->ptr.buffer;
    ai->next = tlsf->area_head;
    ai->end = lb0;
    tlsf->area_head = ai;
    free_ex(b0->ptr.buffer, mem_pool);
    return (b0->size & BLOCK_SIZE);  /*返回新增内存大小*/
}


/* 下面两个函数用于查询，动态内存的使用情况*/
/******************************************************************/
size_t get_used_size(void *mem_pool)
{
/******************************************************************/
#if TLSF_STATISTIC
    return ((tlsf_t *) mem_pool)->used_size;
#else
    return 0;
#endif
}

/******************************************************************/
size_t get_max_size(void *mem_pool)
{
/******************************************************************/
#if TLSF_STATISTIC
    return ((tlsf_t *) mem_pool)->max_size;
#else
    return 0;
#endif
}

/* 内存池销毁函数*/
/******************************************************************/
void destroy_memory_pool(void *mem_pool)
{
/******************************************************************/
    tlsf_t *tlsf = (tlsf_t *) mem_pool;

    tlsf->tlsf_signature = 0; /* 用来表示内存区销毁*/

    TLSF_DESTROY_LOCK(&tlsf->lock);  /* 操作系统函数相关，或自定义函数*/

}


/* 函数功能：tlsf内存分配函数
   形参：   size  所需内存的大小
   返回：   viod *  （无符号指针）。分配成功后，返回内存块的指针ret；分配失败返回NULL。
*/
/******************************************************************/
void *tlsf_malloc(size_t size)
{
/******************************************************************/
    void *ret;

#if USE_MMAP || USE_SBRK
    if (!mp) { /* 如果分配内存块时，没有初始化一个内存区，可以使用以下函数得到一个内存区，并初始化此内存区*/
        size_t area_size;
        void *area;

        area_size = sizeof(tlsf_t) + BHDR_OVERHEAD * 8; /* Just a safety constant */
        area_size = (area_size > DEFAULT_AREA_SIZE) ? area_size : DEFAULT_AREA_SIZE;
        area = get_new_area(&area_size);
        if (area == ((void *) ~0))
            return NULL;        /* Not enough system memory */
        init_memory_pool(area_size, area);
    }
#endif

    TLSF_ACQUIRE_LOCK(&((tlsf_t *)mp)->lock); /*获取上锁，与操作系统有关*/

    ret = malloc_ex(size, mp);

    TLSF_RELEASE_LOCK(&((tlsf_t *)mp)->lock); /*获取解锁，与操作系统有关*/
		
		if (ret == NULL)
				mem_errorno = 0x01;

    return ret;
}

/* 函数功能：tlsf内存释放函数
   形参：   ptr  所需内存的首地址指针
   返回：   viod 无返回
*/
/******************************************************************/
void tlsf_free(void *ptr)
{
/******************************************************************/

    TLSF_ACQUIRE_LOCK(&((tlsf_t *)mp)->lock);  /*上锁，与操作系统有关*/

    free_ex(ptr, mp);

    TLSF_RELEASE_LOCK(&((tlsf_t *)mp)->lock); /*解锁，与操作系统有关*/

}

/* 函数功能：内存重新分配，对ptr所指向的内存块，分配size大小的内存块，即对原内存大小扩充
   形参：   ptr  原内存的首地址指针    size  所需内存的大小，也就是扩充之后内存的大小
   返回：   viod *指针类型；   如果重新分配成功则返回指向被分配内存的指针，否则返回空指针NULL
*/
/******************************************************************/
void *tlsf_realloc(void *ptr, size_t size)
{
/******************************************************************/
    void *ret;

#if USE_MMAP || USE_SBRK
	if (!mp) {
		return tlsf_malloc(size);
	}
#endif

    TLSF_ACQUIRE_LOCK(&((tlsf_t *)mp)->lock);

    ret = realloc_ex(ptr, size, mp);

    TLSF_RELEASE_LOCK(&((tlsf_t *)mp)->lock);

    return ret;
}

/* 函数功能：在内存的动态存储区中分配n个长度为size的连续空间，
             函数返回一个指向分配起始地址的指针；如果分配不成功，返回NULL。
             calloc在动态分配完内存后，自动初始化该内存空间为零，而malloc不初始化，里边数据是随机的垃圾数据
   形参   :  size_t nelem  分配单元的个数   size_t elem_size  每单元大小（btye）
   返回   ： 返回分配的内存块地址指针，若分配失败，则返回NULL    
*/

/******************************************************************/
void *tlsf_calloc(size_t nelem, size_t elem_size)
{
/******************************************************************/
    void *ret;

    TLSF_ACQUIRE_LOCK(&((tlsf_t *)mp)->lock);

    ret = calloc_ex(nelem, elem_size, mp);

    TLSF_RELEASE_LOCK(&((tlsf_t *)mp)->lock);  

    return ret;
}

/* 函数功能：ex内存分配函数，实际内存分配函数
   形参：   size  所需内存的大小； men_pool  内存池的首地址
   返回：   viod *  （无符号指针）。分配成功后，返回内存块的指针ret；分配失败返回NULL。
*/
/******************************************************************/
void *malloc_ex(size_t size, void *mem_pool)
{
/******************************************************************/
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    bhdr_t *b, *b2, *next_b;
    int fl, sl;
    size_t tmp_size;

	/*  调整size值，最小为MIN_BLOCK_SIZE，最小（sizeof(free_ptr_t)）*/
    size = (size < MIN_BLOCK_SIZE) ? MIN_BLOCK_SIZE : ROUNDUP_SIZE(size);

    /* Rounding up the requested size and calculating fl and sl */
    MAPPING_SEARCH(&size, &fl, &sl);  /* 查找满足所需内存大小的一级与二级索引，size的值被调整为所需状态*/

    /* Searching a free block, recall that this function changes the values of fl and sl,
       so they are not longer valid when the function fails */
    b = FIND_SUITABLE_BLOCK(tlsf, &fl, &sl);  /* 根据fl与sl的值得到适合的内存块的链表的表头*/
	
	/* 以下部分是用于当前内存池中，没有所需内存块时，从内存中得到新的内存区（使用sbrk or mmap函数）*/
#if USE_MMAP || USE_SBRK
    if (!b) {
        size_t area_size;
        void *area;
        /* Growing the pool size when needed */
        area_size = size + BHDR_OVERHEAD * 8;   /* size plus enough room for the requered headers. */
        area_size = (area_size > DEFAULT_AREA_SIZE) ? area_size : DEFAULT_AREA_SIZE; /* area_size最小值为DEFAULT_AREA_SIZE，避免分割出太多的小内存区*/
        area = get_new_area(&area_size);        /* Call sbrk or mmap */
        if (area == ((void *) ~0))
            return NULL;        /* Not enough system memory */
        add_new_area(area, area_size, mem_pool);
        /* Rounding up the requested size and calculating fl and sl */
        MAPPING_SEARCH(&size, &fl, &sl);
        /* Searching a free block */
        b = FIND_SUITABLE_BLOCK(tlsf, &fl, &sl);
    }
#endif
    if (!b)    /* 如果b空闲链表表头为NULL，表示分配内存失败！*/
        return NULL;            /* Not found */

    EXTRACT_BLOCK_HDR(b, tlsf, fl, sl);  /* 根据一级与二级索引值，从相应链表中得到内存块，并调整bitmap位图*/
    /*-- found: */
    next_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE); /* 根据b->size得到next的物理相邻内存块*/
    /* Should the block be split? */
    tmp_size = (b->size & BLOCK_SIZE) - size;  /* 分割所需内存，得到剩余内存大小*/
    if (tmp_size >= sizeof(bhdr_t)) { /* 分割后的剩余内存值大于等于sizeof(bhdr_t)值，即大于等于一个块头*/
        tmp_size -= BHDR_OVERHEAD;    
        b2 = GET_NEXT_BLOCK(b->ptr.buffer, size); /* 得到剩余内存块的地址*/
        b2->size = tmp_size | FREE_BLOCK | PREV_USED;  /* 为分割下来的内存块的size赋值*/
        next_b->prev_hdr = b2;            /* next_b内存块链接相邻的前一个物理内存块*/
        MAPPING_INSERT(tmp_size, &fl, &sl); /* 查找剩余内存块的空闲链表的一级与二级索引值*/
        INSERT_BLOCK(b2, tlsf, fl, sl);    /*  插入内存块，且总是查入表头*/
       
		/* 更新b2块的前一块的内存地址，*/
       /*add by vector,right?*/ b2->prev_hdr = b;
		
		/*  size后两位更新，只把0bit改为USED_BLOCK*/
        b->size = size | (b->size & PREV_STATE); /* 参数size为所需内存大小，更新b块的状态*/ 
    } else {      /* 所得内存块不需要分割，只需更新size的后两位*/
        next_b->size &= (~PREV_FREE);   
        b->size &= (~FREE_BLOCK);       /* Now it's used */
    }

    TLSF_ADD_SIZE(tlsf, b);
		
		if (tlsf->used_size > DM_MEM_SIZE)
			mem_errorno = 0x03;

    return (void *) b->ptr.buffer;
}

/* 函数功能：释放ftr所在的内存块，并根据情况合并前后内存块，更新相应bitmap标志位
   形参：   ptr  释放内存指针； men_pool  内存池的首地址
   返回：   viod *  （无符号指针）。分配成功后，返回内存块的指针ret；分配失败返回NULL。
*/
/* */
/******************************************************************/
void free_ex(void *ptr, void *mem_pool)
{
/******************************************************************/
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    bhdr_t *b, *tmp_b;
    int fl = 0, sl = 0;

    if (!ptr) {   /*ptr为NULL，直接返回*/
        return;
    }
    b = (bhdr_t *) ((char *) ptr - BHDR_OVERHEAD);
    b->size |= FREE_BLOCK; /* 所释放内存块状态更新（size后两位更新）*/

    TLSF_REMOVE_SIZE(tlsf, b);  /*  #if TLSF_STATISTIC */

    b->ptr.free_ptr.prev = NULL;
    b->ptr.free_ptr.next = NULL;
    tmp_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE); /* 得到b块后面的相邻物理块指针*/
    if (tmp_b->size & FREE_BLOCK) { /*  b块后面块是free的？其后内存块free则合并内存*/
        MAPPING_INSERT(tmp_b->size & BLOCK_SIZE, &fl, &sl); /* 根据tmp_b大小求出一级与二级索引值*/
        EXTRACT_BLOCK(tmp_b, tlsf, fl, sl); /*  提取内存块，并根据内存块在链表中的位置调整空闲链表与位图标志位*/
        b->size += (tmp_b->size & BLOCK_SIZE) + BHDR_OVERHEAD;  /* 把b（ptr）后面的内存块合并到b内存块中，size更新*/
    }
    if (b->size & PREV_FREE) {  /* b块前一块free？free则与前面的内存块合并*/
        tmp_b = b->prev_hdr;    /* 得到b块前1物理块 */
        MAPPING_INSERT(tmp_b->size & BLOCK_SIZE, &fl, &sl);
        EXTRACT_BLOCK(tmp_b, tlsf, fl, sl);
        tmp_b->size += (b->size & BLOCK_SIZE) + BHDR_OVERHEAD;
        b = tmp_b;   /* 更新b指针的值，即b指向合并后的内存块地址*/
    }
    MAPPING_INSERT(b->size & BLOCK_SIZE, &fl, &sl); /**/
    INSERT_BLOCK(b, tlsf, fl, sl);  /*  把释放的内存块插入相应链表的表头*/

    tmp_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    tmp_b->size |= PREV_FREE;    /* 更新后一块的信息，以表示释放的内存块空闲的*/
    tmp_b->prev_hdr = b;         /*  更新后一块内存块的物理块prev_hdr*/ 
		
		if (tlsf->used_size > DM_MEM_SIZE)
			mem_errorno = 0x02;
}


/* 函数功能：内存扩充函数
   形参：   ptr原内存块的指针地址； new_size  扩充后内存的大小； men_pool  内存池的首地址
   返回：   viod *  （无符号指针）。分配成功后，返回内存块的指针；分配失败返回NULL。
*/
/******************************************************************/
void *realloc_ex(void *ptr, size_t new_size, void *mem_pool)
{
/******************************************************************/
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    void *ptr_aux;
    unsigned int cpsize;
    bhdr_t *b, *tmp_b, *next_b;
    int fl, sl;
    size_t tmp_size;

    if (!ptr) {  /* 如果ptr为NULL*/
        if (new_size)  /* 并且new_size不为0，realloc函数等同malloc函数使用*/
            return (void *) malloc_ex(new_size, mem_pool);
        if (!new_size) /* 如果new_size为0，返回NULL*/
            return NULL;
    } else if (!new_size) { /*  如果ptr不等于NULL，并且new_size不为0，reallo函数等同free函数使用，并返回NULL*/
        free_ex(ptr, mem_pool);
        return NULL;
    }

    b = (bhdr_t *) ((char *) ptr - BHDR_OVERHEAD);
    next_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    new_size = (new_size < MIN_BLOCK_SIZE) ? MIN_BLOCK_SIZE : ROUNDUP_SIZE(new_size); /* 新内存大小调整，8bit对齐*/
    tmp_size = (b->size & BLOCK_SIZE);   /* 原内存块大小*/
    if (new_size <= tmp_size) {  /*如果原内存块大小大于等于所需新内存的大小*/
	   TLSF_REMOVE_SIZE(tlsf, b);  /* 统计函数相关*/
 	   if (next_b->size & FREE_BLOCK) {  /*如果其后的内存块是free的*/
            MAPPING_INSERT(next_b->size & BLOCK_SIZE, &fl, &sl);  /*得到next内存块的fl与sl值*/
            EXTRACT_BLOCK(next_b, tlsf, fl, sl);                  /* 根据fl，sl的值提取next_block内存块，并更新bitmap位图*/
            tmp_size += (next_b->size & BLOCK_SIZE) + BHDR_OVERHEAD;  /* */
            next_b = GET_NEXT_BLOCK(next_b->ptr.buffer, next_b->size & BLOCK_SIZE);
            /* We allways reenter this free block because tmp_size will
               be greater then sizeof (bhdr_t) */
        }
        tmp_size -= new_size;
        if (tmp_size >= sizeof(bhdr_t)) {
            tmp_size -= BHDR_OVERHEAD;
            tmp_b = GET_NEXT_BLOCK(b->ptr.buffer, new_size);
            tmp_b->size = tmp_size | FREE_BLOCK | PREV_USED;
            next_b->prev_hdr = tmp_b;
            next_b->size |= PREV_FREE;
            MAPPING_INSERT(tmp_size, &fl, &sl);
            INSERT_BLOCK(tmp_b, tlsf, fl, sl);
            b->size = new_size | (b->size & PREV_STATE);
        }
	TLSF_ADD_SIZE(tlsf, b);
        return (void *) b->ptr.buffer;
    }
    if ((next_b->size & FREE_BLOCK)) { /* 如果新size大于原size，并且后一块free */
        if (new_size <= (tmp_size + (next_b->size & BLOCK_SIZE))) { /* 若后面空闲内存块够用，则从其后的空闲内存块中分配一块即可*/
			TLSF_REMOVE_SIZE(tlsf, b);
            MAPPING_INSERT(next_b->size & BLOCK_SIZE, &fl, &sl);
            EXTRACT_BLOCK(next_b, tlsf, fl, sl);
            b->size += (next_b->size & BLOCK_SIZE) + BHDR_OVERHEAD;
            next_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
            next_b->prev_hdr = b;
            next_b->size &= ~PREV_FREE;
            tmp_size = (b->size & BLOCK_SIZE) - new_size;
            if (tmp_size >= sizeof(bhdr_t)) { /* 其后分割剩余的内存块大于sizeof(bhdr_t)，而为其组织为一个空闲块*/
                tmp_size -= BHDR_OVERHEAD;
                tmp_b = GET_NEXT_BLOCK(b->ptr.buffer, new_size);
                tmp_b->size = tmp_size | FREE_BLOCK | PREV_USED;
                next_b->prev_hdr = tmp_b;
                next_b->size |= PREV_FREE;
                MAPPING_INSERT(tmp_size, &fl, &sl);
                INSERT_BLOCK(tmp_b, tlsf, fl, sl);
                b->size = new_size | (b->size & PREV_STATE);
            }
			TLSF_ADD_SIZE(tlsf, b);
            return (void *) b->ptr.buffer;
        }
    }

  /* 如果其后没有空闲块，或者其后的空闲块大小不够用，
	则利用malloc函数从内存池中重新分配一块new_size大小的内存块
	*/
    if (!(ptr_aux = malloc_ex(new_size, mem_pool))){ 
        return NULL;
    }      
    
    cpsize = ((b->size & BLOCK_SIZE) > new_size) ? new_size : (b->size & BLOCK_SIZE);  /* 调整cpsize值，即复制的字节数*/

    memcpy(ptr_aux, ptr, cpsize); /* 把ptr中cpsize字节的数据复制到prt_aux处*/

    free_ex(ptr, mem_pool);  /* 如果前面的if都不成立，则执行释放原内存块*/
    return ptr_aux;          /* 返回调整后的内存块的指针*/
}


/* 函数功能：calloc函数
   形参：   nelem  分配单元个数；eleme_size 单元大小； men_pool  内存池的首地址
   返回：   viod *  （无符号指针）。分配成功后，返回内存块的指针；分配失败返回NULL。
*/
/******************************************************************/
void *calloc_ex(size_t nelem, size_t elem_size, void *mem_pool)
{
/******************************************************************/
    void *ptr;

    if (nelem <= 0 || elem_size <= 0)
        return NULL;

    if (!(ptr = malloc_ex(nelem * elem_size, mem_pool)))  /* 实际分配过程与malloc相同，使用malloc_ex函数分配*/
        return NULL;
    memset(ptr, 0, nelem * elem_size); /* 分配成功后的内存块清零*/

    return ptr;
}

void dm_init(void)
{
	init_memory_pool (DM_MEM_SIZE, work_mem);
}


#if _DEBUG_TLSF_

/***************  DEBUG FUNCTIONS  调试函数 **************/

/* The following functions have been designed to ease the debugging of */
/* the TLSF  structure.  For non-developing  purposes, it may  be they */
/* haven't too much worth.  To enable them, _DEBUG_TLSF_ must be set.  */
/*                      要使用调试函数 需要 _DEBUG_TLSF_ 置1           */
  
extern void dump_memory_region(unsigned char *mem_ptr, unsigned int size);
extern void print_block(bhdr_t * b);
extern void print_tlsf(tlsf_t * tlsf);
void print_all_blocks(tlsf_t * tlsf);

void dump_memory_region(unsigned char *mem_ptr, unsigned int size)
{

    unsigned long begin = (unsigned long) mem_ptr;
    unsigned long end = (unsigned long) mem_ptr + size;
    int column = 0;

    begin >>= 2;
    begin <<= 2;

    end >>= 2;
    end++;
    end <<= 2;

    PRINT_MSG("\nMemory region dumped: 0x%lx - 0x%lx\n\n", begin, end);

    column = 0;
    PRINT_MSG("0x%lx ", begin);

    while (begin < end) {
        if (((unsigned char *) begin)[0] == 0)
            PRINT_MSG("00");
        else
            PRINT_MSG("%02x", ((unsigned char *) begin)[0]);
        if (((unsigned char *) begin)[1] == 0)
            PRINT_MSG("00 ");
        else
            PRINT_MSG("%02x ", ((unsigned char *) begin)[1]);
        begin += 2;
        column++;
        if (column == 8) {
            PRINT_MSG("\n0x%lx ", begin);
            column = 0;
        }

    }
    PRINT_MSG("\n\n");
}

void print_block(bhdr_t * b)
{
    if (!b)
        return;
    PRINT_MSG(">> [%p] (", b);
    if ((b->size & BLOCK_SIZE))
        PRINT_MSG("%lu bytes, ", (unsigned long) (b->size & BLOCK_SIZE));
    else
        PRINT_MSG("sentinel, ");
    if ((b->size & BLOCK_STATE) == FREE_BLOCK)
        PRINT_MSG("free [%p, %p], ", b->ptr.free_ptr.prev, b->ptr.free_ptr.next);
    else
        PRINT_MSG("used, ");
    if ((b->size & PREV_STATE) == PREV_FREE)
        PRINT_MSG("prev. free [%p])\n", b->prev_hdr);
    else
        PRINT_MSG("prev used)\n");
}

void print_tlsf(tlsf_t * tlsf)
{
    bhdr_t *next;
    int i, j;

    PRINT_MSG("\nTLSF at %p\n", tlsf);

    PRINT_MSG("FL bitmap: 0x%x\n\n", (unsigned) tlsf->fl_bitmap);

    for (i = 0; i < REAL_FLI; i++) {
        if (tlsf->sl_bitmap[i])
            PRINT_MSG("SL bitmap 0x%x\n", (unsigned) tlsf->sl_bitmap[i]);
        for (j = 0; j < MAX_SLI; j++) {
            next = tlsf->matrix[i][j];
            if (next)
                PRINT_MSG("-> [%d][%d]\n", i, j);
            while (next) {
                print_block(next);
                next = next->ptr.free_ptr.next;
            }
        }
    }
}

void print_tlsf_xbl(void)
{
     tlsf_t *tlsf;
     bhdr_t *next;
    int i, j;
    tlsf = (tlsf_t *) work_mem;
    PRINT_MSG("\nTLSF at %p\n", tlsf);

    PRINT_MSG("FL bitmap: 0x%x\n\n", (unsigned) tlsf->fl_bitmap);

    for (i = 0; i < REAL_FLI; i++) {
        if (tlsf->sl_bitmap[i])
            PRINT_MSG("SL bitmap 0x%x\n", (unsigned) tlsf->sl_bitmap[i]);
        for (j = 0; j < MAX_SLI; j++) {
            next = tlsf->matrix[i][j];
            if (next)
                PRINT_MSG("-> [%d][%d]\n", i, j);
            while (next) {
                print_block(next);
                next = next->ptr.free_ptr.next;
            }
        }
    }

}
void print_all_blocks(tlsf_t * tlsf)
{
    area_info_t *ai;
    bhdr_t *next;
    PRINT_MSG("\nTLSF at %p\nALL BLOCKS\n\n", tlsf);
    ai = tlsf->area_head;
    while (ai) {
        next = (bhdr_t *) ((char *) ai - BHDR_OVERHEAD);
        while (next) {
            print_block(next);
            if ((next->size & BLOCK_SIZE))
                next = GET_NEXT_BLOCK(next->ptr.buffer, next->size & BLOCK_SIZE);
            else
                next = NULL;
        }
        ai = ai->next;
    }
}

void print_all_blocks_xbl(void)
{
    tlsf_t *tlsf;
    tlsf = (tlsf_t *) work_mem;
    area_info_t *ai;
    bhdr_t *next;
    PRINT_MSG("\nTLSF at %p\nALL BLOCKS\n\n", tlsf);
    ai = tlsf->area_head;
    while (ai) {
        next = (bhdr_t *) ((char *) ai - BHDR_OVERHEAD);
        while (next) {
            print_block(next);
            if ((next->size & BLOCK_SIZE))
                next = GET_NEXT_BLOCK(next->ptr.buffer, next->size & BLOCK_SIZE);
            else
                next = NULL;
        }
        ai = ai->next;
    }
}

#endif
