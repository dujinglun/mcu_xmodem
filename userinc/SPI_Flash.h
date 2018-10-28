#ifndef __SPI_FLASH_H 
#define __SPI_FLASH_H

#define		SPI_TFE					(1<<0)
#define		SPI_TNF					(1<<1)
#define		SPI_RNE					(1<<2)
#define		SPI_RFF					(1<<3)
#define		SPI_BSY					(1<<4)
#define		SPI_PORT				1
#define		SPI_NSS					23
#define		FLASH_NSSLow()		  (LPC_GPIO->CLR[SPI_PORT] = 1<<SPI_NSS)
#define		FLASH_NSSHigh()		  (LPC_GPIO->SET[SPI_PORT] = 1<<SPI_NSS)

                                                     
#define 	FLASH_ID 				 0XBF41//SST25VF016¶ÁÐ´
#define 	SST25_ReadData            0x03
#define 	SST25_FastReadData        0x0B
#define 	SST25_4KByte_BlockERASE   0x20
#define 	SST25_32KByte_BlockErase  0x52
#define 	SST25_64KByte_BlockErase  0xD8
#define 	SST25_ChipErase           0xC7
#define 	SST25_ByteProgram         0x02
#define 	SST25_AAI_WordProgram     0xAD
#define 	SST25_ReadStatusReg       0x05
#define 	SST25_EnableWriteStatusReg 0x50
#define 	SST25_WriteStatusReg       0x01
#define 	SST25_WriteEnable          0x06
#define 	SST25_WriteDisable         0x04
#define 	SST25_ManufactDeviceID     0x90
#define 	SST25_JedecDeviceID        0x9F
#define 	SST25_EBSY                 0x70
#define 	SST25_DBSY                 0x80 

extern		void		SPI_FLASH_Init(void);
extern		void 		SPI_Flash_Erase_Sector(uint32_t Dst_Addr);
extern		void 		SPI_Flash_Erase_Chip(void);
extern		void 		Flash_WriteByte(uint8_t* pBuffer,uint32_t WriteAddr);
extern		void 		SPI_Flash_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
extern		void 		AutoAddressIncrement_WordProgramB(uint8_t state,uint8_t Byte1, uint8_t Byte2);
extern		void 		AutoAddressIncrement_WordProgramA(uint8_t Byte1, uint8_t Byte2, uint32_t Addr);
extern		void 		SPI_Flash_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);
extern		uint16_t 	SPI_Flash_ReadID(void);
extern		void 		SPI_FLASH_Write_SR(uint8_t sr);
extern		uint8_t 	SPI_Flash_ReadSR(void);
extern		void		SPI_Flash_Config(void);
extern		uint8_t		SPI_Flash_ReadWriteData(uint8_t Data);


extern		void	SPI_FLASH_Write_Disable(void);
extern		void	SPI_FLASH_Write_Enable(void);
extern		void	SST25V_DBSY(void);
extern		void	SST25V_EBSY(void);	
#define		SPI_Flash_Wait_Busy()  			while ((SPI_Flash_ReadSR()&0x01)==0x01)

#endif


