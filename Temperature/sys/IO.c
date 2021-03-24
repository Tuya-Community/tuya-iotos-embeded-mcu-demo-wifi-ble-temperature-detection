#include "IO.h"
#include "delay.h"
#include "TIM.h"
#include "math.h"
#include "stdlib.h"
// 2�� 1ͨ�� 1����  1��ˮ 1��ˮ 1��ʪ 2��IIC ��ʪ�� ���� 2��ADC  ˮλ ����ʪ��
#if 1//IIC
void IIC_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
}
void IIC_Start(void)//����IIC��ʼ�ź�
{
	IIC_SDA_OUT;     //sda�����
	IIC_SDA_SET;//IIC_SDA=1;	  	  
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}
void IIC_Stop(void)//����IICֹͣ�ź�
{
	IIC_SDA_OUT;//sda�����
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//����I2C���߽����ź�				   	
}
void IIC_Ack(void)//����ACKӦ��
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//������ACKӦ��	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//�ȴ�Ӧ���źŵ���:1,����Ӧ��ʧ��;0,����Ӧ��ɹ�
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA����Ϊ����     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//���SDA�Ƿ���Ϊ�ߵ�ƽ
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET;//IIC_SCL=0;
	return 0;  
} 
void IIC_Send_Byte(uint8_t txd)//IIC����һ���ֽ�; �ȷ��͸�λ
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//��һ���ֽڣ��ɼ��Ƿ�Ӧ��λ,1��ack��0����ack �Ӹ�λ��ʼ��
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL_RESET;// IIC_SCL=0; 
		delay_us(5);
		IIC_SCL_SET;//IIC_SCL=1;
		receive<<=1;
		if(IIC_SDA_State)
			receive++;   
		delay_us(5); 
	}					 
		if (ack)
			IIC_Ack(); //����ACK
		else
			IIC_NAck();//����nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//ֱ��дһ���ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //�����ֽ�							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//���ֽ�
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//���͵�ַ	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //�������ģʽ			   
	ret |= IIC_Wait_Ack();
	while(NumToRead)
	{
		if(NumToRead==1)
		{
			*pBuffer=IIC_Read_Byte(0);	
		}
		else
		{
			*pBuffer=IIC_Read_Byte(1);
		}
		pBuffer++;
		NumToRead--;
	}
	IIC_Stop();//����һ��ֹͣ����	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//��һ��д����ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //�����ֽ�							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	delay_us(10);
	return ret;
}
#endif

/******************************************************��ʪ��******************************************************/
uint8_t F_TASK_SHT21=0;
struct ctrl_state air_temp;
#if USING_SHT21//��ʪ��
uint8_t SHT2x_SoftReset()
{
  uint8_t  error=0;           //error variable

  IIC_Start();
  IIC_Send_Byte (I2C_ADR_W); // I2C Adr
	error |= IIC_Wait_Ack();
  IIC_Send_Byte (SOFT_RESET);// Command
	error |= IIC_Wait_Ack();
  IIC_Stop();

  delay_ms(15); // wait till sensor has restarted

  return error;
}
const uint16_t POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  uint8_t crc = 0;	
  uint8_t byteCtr;
	uint8_t bit;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) 
		return 1;
  else 
		return 0;
}
uint8_t SHT2x_ReadUserRegister(uint8_t *pRegisterValue)
{
  uint8_t checksum;   //variable for checksum byte
  uint8_t error=0;    //variable for error code

  IIC_Start();
  IIC_Send_Byte (I2C_ADR_W);
	error |= IIC_Wait_Ack();
  IIC_Send_Byte(USER_REG_R);
	error |= IIC_Wait_Ack();
	
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_R);
	error |= IIC_Wait_Ack();
  *pRegisterValue = IIC_Read_Byte(1);
  checksum=IIC_Read_Byte(0);
  error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
  IIC_Stop();
  return error;
}
uint8_t SHT2x_WriteUserRegister(uint8_t *pRegisterValue)
{
  uint8_t error=0;   //variable for error code

  IIC_Start();
  IIC_Send_Byte (I2C_ADR_W);
	error |= IIC_Wait_Ack();
  IIC_Send_Byte (USER_REG_W);
	error |= IIC_Wait_Ack();
  IIC_Send_Byte (*pRegisterValue);
	error |= IIC_Wait_Ack();
  IIC_Stop();
  return error;
}
uint8_t SHT2x_ResolutionSet(uint8_t mode)
{
	uint8_t error=0;
	uint8_t userRegister=0;
	error |= SHT2x_ReadUserRegister(&userRegister);
	userRegister = (userRegister & ~SHT2x_RES_MASK) | mode;
	error |= SHT2x_WriteUserRegister(&userRegister);
	return error;
}
uint8_t SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
{
  uint8_t  checksum;   //checksum
  uint8_t  data[2];    //data array for checksum verification
  uint8_t  error=0;    //error variable
  //-- write I2C sensor address and command --
  IIC_Start();
  IIC_Send_Byte (I2C_ADR_W); // I2C Adr
	error |= IIC_Wait_Ack();
  switch(eSHT2xMeasureType)
  { 
		case HUMIDITY: IIC_Send_Byte (TRIG_RH_MEASUREMENT_HM); break;
    case TEMP    : IIC_Send_Byte (TRIG_T_MEASUREMENT_HM);  break;
    default: return 1;
  }
	error += IIC_Wait_Ack();
	
  //-- wait until hold master is released --
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_R);
	error += IIC_Wait_Ack()*10;
  //-- read two data bytes and one checksum byte --
  delay_ms(95);
	

  pMeasurand->s16.u8H = data[0] = IIC_Read_Byte(1);
  pMeasurand->s16.u8L = data[1] = IIC_Read_Byte(1);
  checksum=IIC_Read_Byte(0);
	IIC_Stop();
  //-- verify checksum --
  error += SHT2x_CheckCrc (data,2,checksum)*100;
  
  return error;
}
uint8_t SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
{
  uint8_t  checksum;   //checksum
  uint8_t  data[2];    //data array for checksum verification
  uint8_t  error=0;    //error variable
  uint16_t i=0;        //counting variable

  //-- write I2C sensor address and command --
  IIC_Start();
  IIC_Send_Byte (I2C_ADR_W); // I2C Adr
  switch(eSHT2xMeasureType)
  { case HUMIDITY: IIC_Send_Byte (TRIG_RH_MEASUREMENT_POLL); break;
    case TEMP    : IIC_Send_Byte (TRIG_T_MEASUREMENT_POLL);  break;
    default: return 1;
  }
  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
  do
  { IIC_Start();
    delay_ms(10);  //delay 10ms
		IIC_Send_Byte (I2C_ADR_R); 
    if(i++ >= 20) break;
  } while(IIC_Wait_Ack()== 0);
  if (i>=20) error |= 1;

  //-- read two data bytes and one checksum byte --
  pMeasurand->s16.u8H = data[0] = IIC_Read_Byte(1);
  pMeasurand->s16.u8L = data[1] = IIC_Read_Byte(1);
  checksum=IIC_Read_Byte(0);

  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  IIC_Stop();

  return error;
}

float SHT2x_CalcTemperatureC(uint16_t uint16_tsT)
{
  float temperatureC;            // variable for result

  uint16_tsT &= ~0x0003;           // clear bits [1..0] (status bits)
  
  //-- calculate temperature [�C] --
  temperatureC=  175.72/65536 *(float)uint16_tsT - 46.85; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}
float SHT2x_CalcRH(uint16_t uint16_tsRH)
{
  float humidityRH;              // variable for result

  uint16_tsRH &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --

  humidityRH = -6.0 + 125.0/65536 * (float)uint16_tsRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}
void SHT21_Init(void)
{
	uint8_t error=0; // reset error status
	
	// --- Reset sensor by command ---
	error += SHT2x_SoftReset();
	error += SHT2x_ResolutionSet(SHT2x_RES_12_14BIT);
}
void TASK_SHT21(void)
{
	uint8_t error=0;   
	nt16 sT;
	unsigned long temperatureC;
	
	error += SHT2x_MeasureHM(TEMP, &sT);
	air_temp.now = SHT2x_CalcTemperatureC(sT.u16);
	temperatureC =(unsigned long)(air_temp.now*10.0);	
	mcu_dp_value_update(DPID_TEMP_CURRENT,temperatureC);

}

#endif

void Modules_Init(void)
{
	IIC_Init();
	SHT21_Init();
}
void SwitchIO_Init(void)
{
	LED_4_OUT;	
}
void IO_Init(void)
{
	Modules_Init();
	SwitchIO_Init();
}
