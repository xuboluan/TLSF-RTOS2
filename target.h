#ifndef _TARGET_H_
#define _TARGET_H_


#define TLSF_MLOCK_T            osMutexId_t
#define TLSF_CREATE_LOCK(l)     {(*l) = osMutexNew(&TLSF_Mutex_attr);}
#define TLSF_DESTROY_LOCK(l)    {osMutexDelete(*l);}

#define TLSF_ACQUIRE_LOCK(l)    {if (__get_IPSR() != 0U) {} else {osMutexAcquire((*l), osWaitForever);}}
	
#define TLSF_RELEASE_LOCK(l)    { \
	if (__get_IPSR() != 0U) { \
	} \
	else { \
	 \
        osMutexRelease((*l)); \
	} \
}



//#define TLSF_ACQUIRE_LOCK(l)    { \
//	if (__get_IPSR() != 0U) { \
//	} \
//	else { \
//		osMutexAcquire((*l), osWaitForever); \
//		//__disable_irq(); \
//	} \
//}
//	
//#define TLSF_RELEASE_LOCK(l)    { \
//	if (__get_IPSR() != 0U) { \
//	} \
//	else { \
//		__enable_irq(); \
//        osMutexRelease((*l)); \
//	} \
//}

#endif
