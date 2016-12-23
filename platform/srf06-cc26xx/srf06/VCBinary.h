#ifndef VCBINARY_H
#define VCBINARY_H

#ifdef VCBINARY_GLOBALS
#define VCBINARY_EXT
#else
#define VCBINARY_EXT extern
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#if defined (WIN32)
#define CRYPTODLL   __declspec(dllexport)
#else
#define  CRYPTODLL              /**/
#endif 
#ifdef __cplusplus
extern "C" {
#endif


typedef unsigned char BOOL;
#define PASS 0x34
#define true 1
#define false 0
/**
\fn void CreateShares(unsigned char * sourceBuff, unsigned int buffLength, unsigned char *share1Buff, unsigned char *share2Buff);
\brief function to generate VC shares
\param sourceBuff - points to data to be converted to shares
\param buffLength - length of the data to be converted
\param share1Buff - points to first share of data.
\param share2Buff - points to second share of data.
\return Returns void.
*/
CRYPTODLL VCBINARY_EXT void CreateShares(unsigned char * sourceBuff, unsigned int buffLength, unsigned char *share1Buff, unsigned char *share2Buff);
/**
\fn void JoinShares(unsigned char * sourceBuff, unsigned int buffLength, unsigned char *share1Buff, unsigned char *share2Buff);
\brief funtions to join shares
\param sourceBuff - points to joined data
\param buffLength - length of the data to be converted
\param share1Buff - points to first share of data.
\param share2Buff - points to second share of data.
\return Returns void.
*/
CRYPTODLL VCBINARY_EXT void JoinShares(unsigned char * sourceBuff, unsigned int buffLength, unsigned char *share1Buff, unsigned char *share2Buff);
#ifdef __cplusplus
}
#endif
#endif
