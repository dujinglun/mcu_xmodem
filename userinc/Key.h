#ifndef __KEY_H__
#define __KEY_H__
//===================================================================
#define		KEY_PORT		0
#define		KEY_UP_BIT		7
#define		KEY_DN_BIT		8
#define		KEY_OK_BIT		9
#define		KEY_BK_BIT		13

#define		KEY_CHECK		(1 | (1<<7) | (1<<8) | (1<<9) | (1<<13))
#define		KEY_NULL		0
#define		KEY_UP			1
#define		KEY_DN			2
#define		KEY_OK			3
#define		KEY_BK			4		
//===================================================================
extern		void	Key_Init(void);
extern		uint8_t	Key_GetValue(void);
#endif

