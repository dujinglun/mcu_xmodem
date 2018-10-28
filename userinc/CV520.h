#ifndef __CV520_H 
#define __CV520_H

#define		CV520_FACE_PORT				1
#define		CV520_FACE_SEL0_BIT			28
#define		CV520_FACE_SEL1_BIT			31	 //cv520 ֻ��һ�ֽӿ�ģʽ��SPI

#define		IO_OUTPUT					1
#define		IO_INPUT					0

#define		CV520_SYSTEM_PORT			0
#define		CV520_SYSTEM_RST_PD_BIT		7
#define		CV520_SYSTEM_IRQ_BIT		8

#define		CV520_SPI_PORT				1
#define		CV520_SPI_NSS				19
#define		CV520_SPI_SCK				20
#define		CV520_SPI_MISO				21
#define		CV520_SPI_MOSI				22	

#define		SPI_TFE					(1<<0)
#define		SPI_TNF					(1<<1)
#define		SPI_RNE					(1<<2)
#define		SPI_RFF					(1<<3)
#define		SPI_BSY					(1<<4)

#define		M1_LogicAddr2PhysicsAddr(SecNum ,BankNum)		( SecNum * 4 + BankNum)	

void		CV520_HWReset(void);
void		CV520_IOConfig(void);
void		CV520_SPI_Config(void);
uint8_t		CV520_SPI_SendData(uint8_t Data);

void	CV520_Test(void);
void	CV520_Init(void);
void	CV520_PcdReset(void);
uint8_t CV520_PcdSelect(uint8_t *pSnr);
uint8_t CV520_PcdAnticoll(uint8_t *pSnr);
uint8_t	CV520_PcdRequest(uint8_t Req_Code,uint8_t *pTagType);
uint8_t CV520_PcdValue(uint8_t DD_mode,uint8_t Addr,uint8_t *pValue);
uint8_t CV520_PcdAuthState(uint8_t Auth_mode,uint8_t Addr,uint8_t *pKey,uint8_t *pSnr);
uint8_t CV520_PcdBakValue(uint8_t DstAddr,uint8_t SrcAddr);
uint8_t CV520_PcdWrite(uint8_t Addr,uint8_t *pData);
uint8_t CV520_PcdRead(uint8_t Addr,uint8_t *pData);
uint8_t CV520_PcdHalt(void);

void	CV520_FormatData2Buff( int_fast32_t Data , uint8_t BKAddr , uint8_t *Buff);
uint8_t	CV520_FormatBuff2Data(int_fast32_t *Data , uint8_t *Addr);

#define		CV520_TEST		0
#define		MAXLEN			18
typedef		struct
{
  		uint8_t		Cmd;
		uint8_t		WriteToM1[MAXLEN];
		uint8_t		WriteByte;
		uint8_t		CRCByte;
		uint8_t		ReadFromM1[MAXLEN];
		uint8_t		ReadBit;
}PCD_DataFormat;
typedef		struct
{
        //   0x4400 = Mifare_UltraLight
        //   0x0400 = Mifare_One(S50)
        //   0x0200 = Mifare_One(S70)
        //   0x0800 = Mifare_Pro(X)
        //   0x4403 = Mifare_DESFire
		uint8_t		TagType[2];		//��ǩ����
		uint8_t		SerialNum[4];	//����
		uint8_t		SecNum;			//��ǰ������	0~15
		uint8_t		BankNum;		//��ǰ���	0-2 ���ݿ�  3���ƿ�
		uint8_t		PhysicsAddr;	//��������ַ = ������ * 4 + ���
		uint8_t		SecKeyA[6];		//������ԿA
		uint8_t		VisitCtr[4];	//���ʿ���
		uint8_t		SecKeyB[6];		//������ԿB
		uint8_t		DataBuff[16];	//��д���ݻ���
}Card_Infor;
//====================================================================================================
//520������
#define 	PCD_IDLE              0x00               //ȡ����ǰ����
#define 	PCD_AUTHENT           0x0E               //��֤��Կ
#define 	PCD_RECEIVE           0x08               //��������
#define 	PCD_TRANSMIT          0x04               //��������
#define 	PCD_TRANSCEIVE        0x0C               //���Ͳ���������
#define 	PCD_RESETPHASE        0x0F               //��λ
#define 	PCD_CALCCRC           0x03               //CRC����
//===================================================================================================
//M1��Ƭ������
#define 	PICC_REQIDL           0x26               //Ѱ��������δ��������״̬
#define 	PICC_REQALL           0x52               //Ѱ��������ȫ����
#define 	PICC_ANTICOLL1        0x93               //����ײ
#define 	PICC_ANTICOLL2        0x95               //����ײ
#define 	PICC_AUTHENT1A        0x60               //��֤A��Կ
#define 	PICC_AUTHENT1B        0x61               //��֤B��Կ
#define 	PICC_READ             0x30               //����
#define 	PICC_WRITE            0xA0               //д��
#define 	PICC_DECREMENT        0xC0               //�ۿ�
#define 	PICC_INCREMENT        0xC1               //��ֵ
#define 	PICC_RESTORE          0xC2               //�������ݵ�������
#define 	PICC_TRANSFER         0xB0               //���滺����������
#define 	PICC_HALT             0x50               //����
//===================================================================================================
//520 FIFO���ȶ���
#define 	DEF_FIFO_LENGTH       64                 //FIFO size=64byte
//===================================================================================================
//520�Ĵ�������
// PAGE 0
#define     RFU00                 0x00    
#define     CommandReg            0x01    
#define     ComIEnReg             0x02    
#define     DivlEnReg             0x03    
#define     ComIrqReg             0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1     
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAskReg              0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2    
#define     RFU20                 0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3      
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     RFU3C                 0x3C   
#define     RFU3D                 0x3D   
#define     RFU3E                 0x3E   
#define     RFU3F		  		  0x3F
//====================================================================================================
//��520ͨѶʱ���صĴ������
#define 	MI_OK                  0
#define 	MI_NOTAGERR            1
#define 	MI_ERR                 2

#endif 

