#ifndef HQSYS_PCBA
#define HQSYS_PCBA


typedef enum
{
   PCBA_WIFI=0,
   PCBA_ROW_LTE_DATA,
   PCBA_PRC_LTE_DATA,

   PCBA_UNKNOW,


}PCBA_CONFIG;

struct pcba_info
{
	PCBA_CONFIG pcba_config;
	char pcba_name[32];
};

typedef enum
{
   PCBA_STATE_EVT=0,
   PCBA_STATE_DVT1,
   PCBA_STATE_DVT2,
   PCBA_STATE_PVT,
   PCBA_STATE_UNKNOW,


}PCBA_STATE_CONFIG;

struct pcba_state_info
{
	PCBA_STATE_CONFIG pcba_state_config;
	char pcba_state_name[32];
};


PCBA_CONFIG get_huaqin_pcba_config(void);

#endif
