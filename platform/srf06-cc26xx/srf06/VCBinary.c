#define VCBINARY_GLOBALS
#include "VCBinary.h"
typedef unsigned char BOOL;
//#define PASS 0x00
#define true 1
#define false 0
CRYPTODLL void CreateShares(unsigned char * sourceBuff, unsigned int buffLength, unsigned char *share1Buff, unsigned char *share2Buff)
{
	BOOL oddParity = true;
	unsigned int i,j;
	unsigned char temp;
	memset(share1Buff,0x00,buffLength);
	memset(share2Buff,0x00,buffLength);
  	for(i=0;i<buffLength;i++)
	{
		temp = sourceBuff[i] ^ PASS;
  		for (j = 0; j < 8; j++)
        {
   			if ((temp & (0x01 << j)) == 0)
            {
                oddParity = false;
            }
            else
            {
                oddParity = true;
            }
            if ((rand() % 2) == 0)
            {
                share1Buff[i] = (unsigned char)(share1Buff[i] & ~(0x01 << j));
				if (oddParity)
                {
                    share2Buff[i] = (unsigned char)(share2Buff[i] | (0x01 << j));
                }
            }
            else
            {
                share1Buff[i] = (unsigned char)(share1Buff[i] | (0x01 << j));
                if (!oddParity)
                {
                    share2Buff[i] = (unsigned char)(share2Buff[i] | (0x01 << j));
                }
            }
          }
	}
}
CRYPTODLL void JoinShares(unsigned char * sourceBuff, unsigned int buffLength, unsigned char *share1Buff, unsigned char *share2Buff)
{
   unsigned int i;
   for(i=0;i<buffLength;i++)
   {
   		sourceBuff[buffLength - 1 - i] = share1Buff[buffLength - 1 - i] ^ share2Buff[buffLength - 1 - i] ^ PASS ;
   }
}
