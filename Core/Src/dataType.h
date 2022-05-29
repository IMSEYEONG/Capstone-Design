
#ifndef DATATYPE_H_
#define DATATYPE_H_

typedef struct _packet_data{
	
	unsigned char header[4];
	unsigned char size, id;
	unsigned char mode, check;

	int32_t t_point;
	int32_t t_vel;
	int32_t t_deg;
	
}Packet_data_t;


typedef union _packet{
	
	Packet_data_t data;
	unsigned char buffer[sizeof(Packet_data_t)];
	
}Packet_t;



#endif /* DATATYPE_H_* */