#ifndef ___FLASH_H___
#define ___FLASH_H___

#define		SPI_TFE					(1<<0)
#define		SPI_TNF					(1<<1)
#define		SPI_RNE					(1<<2)
#define		SPI_RFF					(1<<3)
#define		SPI_BSY					(1<<4)
#define		SPI_PORT				1
#define		SPI_NSS					23
#define		FLASH_CSL()		   		(LPC_GPIO->CLR[SPI_PORT] = 1<<SPI_NSS)
#define		FLASH_CSH()		   		(LPC_GPIO->SET[SPI_PORT] = 1<<SPI_NSS)

#define		FLASH_Read				0X03
#define		FLASH_FastRead			0X0B
#define		FLASH_4KSectorErase		0X20
#define		FLASH_32KBlockErase		0X52
#define		FLASH_64KBlockErase		0XD8
#define		FLASH_ChipErase			0x60 		//0xc7
#define		FLASH_ByteProgram		0x02
#define		FLASH_AAIWProgram		0xAD
#define		FLASH_RDSR				0X05
#define		FLASH_EWSR				0X50
#define		FLASH_WRSR				0X01
#define		FLASH_WREN				0x06
#define		FLASH_WRDI				0X04
#define		FLASH_RDID				0X90		//0XAB
#define		FLASH_JEDEC_ID			0X9F
#define		FLASH_EBSY				0X70
#define		FLASH_DBSY				0x80
#define		FLASH_ROOM				0XFFFFF		//8Mbit
#define		ENZK_Addr				1024		//英文字库地址
#define		CNZK_Addr				3072		//中文字库地址


void		FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn);
void		FLASH_ReadBytes(uint8_t *pBuff,uint32_t Addr,uint16_t ReadLength);
uint8_t		SPI_TransByte(uint8_t Byte);
void	 	SPI_Init(void);
void		FLASH_OpChipErase(void);
void		FLASH_OpSectorErase(uint32_t Addr);
uint16_t	FLASH_ReadID(void);
uint8_t		FLASH_ReadSR(void);
void		FLASH_WriteSR(uint8_t New);
void		FLASH_Init(void);
void		FLASH_ProtectALL(void);
void		FLASH_IsFree(void);
void		FLASH_OpEWSR(void);
void		FLASH_OpWRDI(void);
void		FLASH_OpWREN(void);
void		FLASH_Test(void);
#define		USE_ALLFUNCTION			0x00	
//以下函数不能使用
#if( USE_ALLFUNCTION  )
	void		FLASH_OpDBSY(void);
	void		FLASH_OpEBSY(void);
	void		FLASH_AAIWPMB(uint8_t Byte1,uint8_t Byte2);
	void		FLASH_AAIWPMA(uint8_t Byte1,uint8_t Byte2,uint32_t Addr);
	void		FLASH_WriteBytes(uint8_t *pBuff,uint32_t Addr,uint16_t Bytes);
#endif

#endif


