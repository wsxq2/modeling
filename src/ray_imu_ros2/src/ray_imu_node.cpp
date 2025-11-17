#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <ray_imu_ros2/msg/imu_data.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <serial/serial.h>
#include <cmath>
#include <fstream>
#include <string>
#include <chrono>

// 保留原始结构和数据定义
#define G  9.80665
#define PACKET_CRC_INITIAL_SEED	0x1d0f
#define RB_OK		1
#define RB_ERROR	0

#define SIZE_PROTOCOL_B4	23
#define SIZE_PROTOCOL_B6	41
#define SIZE_PROTOCOL_C4	41
#define SIZE_PROTOCOL_A1	39

typedef enum {
	PROTOCOL_UNKNOWN = 0,
	PROTOCOL_B4,
	PROTOCOL_B6,
	PROTOCOL_C4,
	PROTOCOL_A1,
} protocol_type;


struct IMU_DATA
{
	double Ax; // Accel x axis
	double Ay; // Accel y axis
	double Az; // Accel z axis
	double Gx; // Gyro x axis
	double Gy; // Gyro y axis
	double Gz; // Gyro z axis
	double Roll; // Roll
	double Pitch; // Pitch
	double Yaw; // Yaw
	double Mx;
	double My;
	double Mz;
	double Bar;
	double Temp; // Temperature
};

typedef struct {
	uint8_t 	*buffer; 	
	uint16_t 	size;		
	uint16_t 	start;		
	uint16_t 	end;		
	uint8_t 	msb_start;	
	uint8_t 	msb_end;		
} type_ring_byte_buffer;

static uint8_t data_buf[512];
type_ring_byte_buffer rx_data_rb;
sensor_msgs::msg::Imu imu;


uint16_t const crc_ccitt_table[256] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8u) ^ crc_ccitt_table[(crc ^ c) & 0xffu];
}

uint16_t crc_ccitt_1(uint8_t const *buffer, uint16_t len, uint16_t crc)
{
	uint16_t crc_high, crc_low;
	while (len--)
	{
		crc_high = (crc >> 8);
		crc_low = (crc << 8);
		crc = crc_low ^ crc_ccitt_table[crc_high ^ *buffer++];
	}
	return crc;
}

uint16_t crc_ccitt(const uint8_t *buf, uint16_t len, uint16_t seed)
{
	int i, j;
	uint16_t crc = seed;  //non-augmented inital value equivalent to

	for (i = 0; i < len; i += 1)
	{
		crc ^= (uint16_t)(buf[i] << 8);
		for (j = 0; j < 8; j += 1)
		{
			if ((crc & 0x8000) != 0)
			{
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			}
			else
			{
				crc = (uint16_t)(crc << 1);
			}
		}
	}

	return crc;
}


void ring_byte_buffer_init(type_ring_byte_buffer *self, uint8_t *pBuffer, uint16_t sizeBuffer)
{
	self->buffer 	= pBuffer;
	self->size 		= sizeBuffer;
	self->start		= 0;
	self->end		= 0;
	self->msb_start	= 0;
	self->msb_end	= 0;
}

uint16_t get_ring_byte_buffer_used_size(const type_ring_byte_buffer *self)
{
	if (self->end > self->start) 
	{
		return self->end - self->start;
	} 
	else if (self->end < self->start) 
	{
		return self->size - self->start + self->end;
	} 
	else if (self->msb_end != self->msb_start) 
	{
		return self->size;
	} 
	else 
	{
		return 0;
	}
}

void ring_byte_buffer_push_byte(type_ring_byte_buffer *self, uint8_t byte)
{
	self->buffer[self->end] = byte;

	if (++self->end == self->size) 
	{
		self->msb_end ^= 1;
		self->end 	  = 0;
	}
}

uint8_t ring_byte_buffer_pop_byte(type_ring_byte_buffer *self)
{
	uint8_t byte;

	byte = self->buffer[self->start];

	if (++self->start == self->size) 
	{
		self->msb_start ^= 1;
		self->start     = 0;
	}

	return byte;
}

uint8_t move_ring_byte_buffer_start(type_ring_byte_buffer *self, uint16_t offset)
{
	uint16_t max_offset = get_ring_byte_buffer_used_size(self);

	if(offset > max_offset)
	{
		return RB_ERROR;
	}
	else
	{
		self->start += offset;
		if(self->start >= self->size)
		{
			self->msb_start ^= 1;
			self->start -= self->size;
		}
	}

	return RB_OK;
}

uint8_t ring_byte_buffer_peep_byte(type_ring_byte_buffer *self, uint16_t offset)
{
	uint16_t start_temp = self->start + offset;

	while(start_temp >= self->size) 
	{
		start_temp -= self->size;
	}
	
	return self->buffer[start_temp];
}

void ring_byte_buffer_push_buffer(type_ring_byte_buffer *self, const void *data, uint16_t len)
{
	const uint8_t *byte;
	uint16_t i;

	byte = (const uint8_t *)data;

	for (i = 0; i < len; ++i) 
	{
		ring_byte_buffer_push_byte(self, byte[i]);
	}
}

void ring_byte_buffer_peep_buffer(type_ring_byte_buffer *self, void *data, uint16_t len, uint16_t offset)
{
	uint8_t *byte;
	uint16_t i;

	byte = (uint8_t *)data;

	for (i = 0; i < len; ++i) 
	{
		byte[i] = ring_byte_buffer_peep_byte(self, offset+i);
	}
}

void ring_byte_buffer_pop_buffer(type_ring_byte_buffer *self, void *data, uint16_t len)
{
	uint8_t *byte;
	uint16_t i;

	byte = (uint8_t *)data;

	for (i = 0; i < len; ++i) 
	{
		byte[i] = ring_byte_buffer_pop_byte(self);
	}
}

uint8_t try_protocol_b4(uint8_t* protcol_buf, IMU_DATA* data, protocol_type* type)
{
	uint16_t valid_size = get_ring_byte_buffer_used_size(&rx_data_rb);
	if(valid_size >= SIZE_PROTOCOL_B4)
	{
		if(*type == PROTOCOL_UNKNOWN || *type == PROTOCOL_B4)
		{
			for(uint16_t i = 0; i<(valid_size - SIZE_PROTOCOL_B4 + 1); i++)//valid_size - SIZE_PROTOCOL_B4 + 1是为了不越界buffer 这个i表示尝试从偏移i字节处，看能不能找到帧头和一整帧数据 类似滑动窗口
			{
				ring_byte_buffer_peep_buffer(&rx_data_rb, protcol_buf, SIZE_PROTOCOL_B4, i);
				if((protcol_buf[0]==0x7F)&&(protcol_buf[1]==0x80))		//B6 head is 7F 80
				{
					uint8_t check_sum = 0;
					for (uint16_t j=2; j<SIZE_PROTOCOL_B4; j++)
					{
						check_sum += protcol_buf[j];
					}
					if (check_sum == 0xff)
					{
						move_ring_byte_buffer_start(&rx_data_rb, SIZE_PROTOCOL_B4+i);

						int16_t sensors[10];

						for(uint16_t k=0; k<10; k++)
						{
							sensors[k] = (int16_t)(((uint16_t)protcol_buf[3+k*2] <<8)|(uint16_t)protcol_buf[2+k*2]);
						}

						data->Ax = sensors[0] / 3276.8;
						data->Ay = sensors[1] / 3276.8;
						data->Az = sensors[2] / 3276.8;
						
						data->Gx = sensors[3] / 2980.1081;
						data->Gy = sensors[4] / 2980.1081;
						data->Gz = sensors[5] / 2980.1081;
						
						data->Roll  = sensors[6] / 10430.37835;
						data->Pitch = sensors[7] / 10430.37835;
						data->Yaw   = sensors[8] / 10430.37835;
						
						data->Temp = sensors[9] / 327.68;
						
						*type = PROTOCOL_B4;
						return 1;
					}
				}
			}

			*type = PROTOCOL_UNKNOWN;
		}
	}
	else
	{
		*type = PROTOCOL_UNKNOWN;
	}

	return (*type == PROTOCOL_UNKNOWN)?0:1;
}


uint8_t try_protocol_b6(uint8_t* protcol_buf, IMU_DATA* data, protocol_type* type)
{
	uint16_t valid_size = get_ring_byte_buffer_used_size(&rx_data_rb);
	if(valid_size >= SIZE_PROTOCOL_B6)
	{
		if(*type == PROTOCOL_UNKNOWN || *type == PROTOCOL_B6)
		{
			for(uint16_t i = 0; i<(valid_size - SIZE_PROTOCOL_B6 + 1); i++)
			{
				ring_byte_buffer_peep_buffer(&rx_data_rb, protcol_buf, SIZE_PROTOCOL_B6, i);
				if((protcol_buf[0]==0x7F)&&(protcol_buf[1]==0x82))		//B6 head is 7F 82
				{
					uint8_t check_sum = 0;
					for (uint16_t j=2; j<SIZE_PROTOCOL_B6; j++)
					{
						check_sum += protcol_buf[j];
					}
					if (check_sum == 0xff)
					{
						move_ring_byte_buffer_start(&rx_data_rb, SIZE_PROTOCOL_B6+i);

						int32_t sensors[10];

						for(uint16_t k=0; k<9; k++)
						{
							sensors[k] = (int32_t)(((uint32_t)protcol_buf[2+k*4] <<24)|((uint32_t)protcol_buf[3+k*4] <<16)|((uint32_t)protcol_buf[4+k*4] <<8)|(uint32_t)protcol_buf[5+k*4]);
						}
						sensors[9] = (int16_t)((uint16_t)protcol_buf[38] + ((uint16_t)protcol_buf[39]<<8));

						data->Ax = sensors[0] / 134217728.0;
						data->Ay = sensors[1] / 134217728.0;
						data->Az = sensors[2] / 134217728.0;
						
						data->Gx = sensors[3] / 134217728.0;
						data->Gy = sensors[4] / 134217728.0;
						data->Gz = sensors[5] / 134217728.0;
						
						data->Roll  = sensors[6] / 536870912.0;
						data->Pitch = sensors[7] / 536870912.0;
						data->Yaw   = sensors[8] / 536870912.0;
						
						data->Temp = sensors[9] / 327.68;
						
						*type = PROTOCOL_B6;
						return 1;
					}
				}
			}
			
			*type = PROTOCOL_UNKNOWN;
		}
	}
	else
	{
		*type = PROTOCOL_UNKNOWN;
	}

	return (*type == PROTOCOL_UNKNOWN)?0:1;
}


uint8_t try_protocol_c4(uint8_t* protcol_buf, IMU_DATA* data, protocol_type* type)
{
	uint16_t valid_size = get_ring_byte_buffer_used_size(&rx_data_rb);
	if(valid_size >= SIZE_PROTOCOL_C4)
	{
		if(*type == PROTOCOL_UNKNOWN || *type == PROTOCOL_C4)
		{
			for(uint16_t i = 0; i<(valid_size - SIZE_PROTOCOL_C4 + 1); i++)
			{
				ring_byte_buffer_peep_buffer(&rx_data_rb, protcol_buf, SIZE_PROTOCOL_C4, i);
				if((protcol_buf[0]==0x7F)&&(protcol_buf[1]==0x94))		//B6 head is 7F 94
				{
					uint8_t check_sum = 0;
					for (uint16_t j=2; j<SIZE_PROTOCOL_C4; j++)
					{
						check_sum += protcol_buf[j];
					}
					if (check_sum == 0xff)
					{
						move_ring_byte_buffer_start(&rx_data_rb, SIZE_PROTOCOL_C4+i);

						int32_t sensors[10];

						for(uint16_t k=0; k<9; k++)
						{
							sensors[k] = (int32_t)(((uint32_t)protcol_buf[2+k*4] <<24)|((uint32_t)protcol_buf[3+k*4] <<16)|((uint32_t)protcol_buf[4+k*4] <<8)|(uint32_t)protcol_buf[5+k*4]);
						}
						sensors[9] = (int16_t)((uint16_t)protcol_buf[38] + ((uint16_t)protcol_buf[39]<<8));

						data->Ax = sensors[0] / 10000.0;
						data->Ay = sensors[1] / 10000.0;
						data->Az = sensors[2] / 10000.0;
						
						data->Gx = sensors[3] / 572957.79513;
						data->Gy = sensors[4] / 572957.79513;
						data->Gz = sensors[5] / 572957.79513;
						
						data->Roll  = sensors[6] / 572957.79513;
						data->Pitch = sensors[7] / 572957.79513;
						data->Yaw   = sensors[8] / 572957.79513;
						
						data->Temp = sensors[9] / 327.68;
						
						*type = PROTOCOL_C4;
						return 1;
					}
				}
			}
			
			*type = PROTOCOL_UNKNOWN;
		}
	}
	else
	{
		*type = PROTOCOL_UNKNOWN;
	}

	return (*type == PROTOCOL_UNKNOWN)?0:1;
}

uint8_t try_protocol_a1(uint8_t* protcol_buf, IMU_DATA* data, protocol_type* type)
{
	uint16_t valid_size = get_ring_byte_buffer_used_size(&rx_data_rb);
	if(valid_size >= SIZE_PROTOCOL_A1)
	{
		if(*type == PROTOCOL_UNKNOWN || *type == PROTOCOL_A1)
		{
			for(uint16_t i = 0; i<(valid_size - SIZE_PROTOCOL_A1 + 1); i++)
			{
				ring_byte_buffer_peep_buffer(&rx_data_rb, protcol_buf, SIZE_PROTOCOL_A1, i);
				if((protcol_buf[0]==0x4C)&&(protcol_buf[1]==0x53)&&(protcol_buf[2]==0x41)&&(protcol_buf[3]==0x31))		//a1 head is 4C 53 41 31
				{
					uint16_t crc = 0;
					crc = crc_ccitt(&protcol_buf[2], 35, 0x1D0F);
					if (crc == (((uint16_t)protcol_buf[37] << 8) | protcol_buf[38]))
					{
						move_ring_byte_buffer_start(&rx_data_rb, SIZE_PROTOCOL_A1+i);

						int16_t sensors[13];

						for(uint16_t k=0; k<13; k++)
						{
							sensors[k] = (int16_t)(((uint16_t)protcol_buf[5+k*2] <<8)|(uint16_t)protcol_buf[6+k*2]);
						}

						data->Ax = sensors[6] / 3276.8;
						data->Ay = sensors[7] / 3276.8;
						data->Az = sensors[8] / 3276.8;
						
						data->Gx = sensors[3] / 2980.1081;
						data->Gy = sensors[4] / 2980.1081;
						data->Gz = sensors[5] / 2980.1081;
						
						data->Roll  = sensors[0] / 10430.37835;
						data->Pitch = sensors[1] / 10430.37835;
						data->Yaw   = sensors[2] / 10430.37835;
						
						data->Mx = sensors[9]  / 4096.0;
						data->My = sensors[10] / 4096.0;
						data->Mz = sensors[11] / 4096.0;
						
						data->Temp = sensors[12] / 327.68;
						
						*type = PROTOCOL_A1;
						return 1;
					}
				}
			}
			
			*type = PROTOCOL_UNKNOWN;
		}
	}
	else
	{
		*type = PROTOCOL_UNKNOWN;
	}

	return (*type == PROTOCOL_UNKNOWN)?0:1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ray_imu_node");

    std::string port, imu_frame_id, imu_topic;
    int cfg_baud_rate = 115200;
    int ros_sample_rate = 200;

    node->declare_parameter("port", "/dev/ttyUSB0");
    node->declare_parameter("imu_baudrate", 115200);
    node->declare_parameter("imu_frame_id", "imu_base");
    node->declare_parameter("imu_topic", "imu");
    node->declare_parameter("ros_sample_rate", 200);

    node->get_parameter("port", port);
    node->get_parameter("imu_baudrate", cfg_baud_rate);
    node->get_parameter("imu_frame_id", imu_frame_id);
    node->get_parameter("imu_topic", imu_topic);
    node->get_parameter("ros_sample_rate", ros_sample_rate);

    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    auto imu_data_pub = node->create_publisher<ray_imu_ros2::msg::IMUData>("imu_data", 10);

    rclcpp::Rate r(ros_sample_rate);

    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(cfg_baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(2);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(node->get_logger(), "Unable to open port: %s\n%s", port.c_str(), e.what());
        return 1;
    }

    if (!ser.isOpen()) {
        RCLCPP_ERROR(node->get_logger(), "Serial port open failed");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Serial Port initialized ok");
    RCLCPP_INFO(node->get_logger(), "IMU start work (ROS2 procedural)...");

    ring_byte_buffer_init(&rx_data_rb, data_buf, sizeof(data_buf));

    // 声明一个系统时间的 Clock
    rclcpp::Clock::SharedPtr sys_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    while (rclcpp::ok()) 
    {
        try {
            if (ser.isOpen())
            {
                while (1)
				{
                  IMU_DATA imuData;
                  uint8_t rx_buf_temp[512] = {0};
                  uint8_t protcol_buf[512] = {0};
                  protocol_type protocol_matched_type = PROTOCOL_UNKNOWN;
                  uint8_t new_frame = 0;

                  int data_count_available = ser.available();

                //   RCLCPP_INFO(node->get_logger(), "data_count_available: %d", data_count_available);
                  if(data_count_available)
                  {
                    // parsed_count++;
                    // RCLCPP_INFO(node->get_logger(), "parse_count = %d", parsed_count);
                    ser.read(rx_buf_temp, data_count_available);
                    ring_byte_buffer_push_buffer(&rx_data_rb, rx_buf_temp, data_count_available);
                  }
                  else
                  {
                    break;
                  }
                  if(!new_frame)
                  {
                    new_frame = try_protocol_c4(protcol_buf, &imuData, &protocol_matched_type);
                  }

                  if(!new_frame)
                  {
                    new_frame = try_protocol_b6(protcol_buf, &imuData, &protocol_matched_type);
                  }

                  if(!new_frame)
                  {
                    new_frame = try_protocol_a1(protcol_buf, &imuData, &protocol_matched_type);
                  }
                  if(!new_frame)
                  {
                    new_frame = try_protocol_b4(protcol_buf, &imuData, &protocol_matched_type);
                  }
                //   RCLCPP_INFO(node->get_logger(), "new_frame: %d, protocol: %d", new_frame, protocol_matched_type);

                  if (new_frame) 
				  {
                    rclcpp::Time measurement_time = sys_clock->now();
                    imu.header.stamp = measurement_time;
                    imu.header.frame_id = imu_frame_id;

                    tf2::Quaternion q;
                    q.setRPY(imuData.Roll, -imuData.Pitch, -imuData.Yaw);
                    imu.orientation = tf2::toMsg(q);
                    std::fill(std::begin(imu.orientation_covariance), std::end(imu.orientation_covariance), 0.0);

                    imu.angular_velocity.x = imuData.Gx;
                    imu.angular_velocity.y = -imuData.Gy;
                    imu.angular_velocity.z = -imuData.Gz;

                    imu.linear_acceleration.x = imuData.Ax * G;
                    imu.linear_acceleration.y = -imuData.Ay * G;
                    imu.linear_acceleration.z = -imuData.Az * G;

                    imu_pub->publish(imu);

                    static double last_time_ms = 0;

                    ray_imu_ros2::msg::IMUData sensor_data;
                    sensor_data.ax_g   =  imuData.Ax;
                    sensor_data.ay_g   = -imuData.Ay;
                    sensor_data.az_g   = -imuData.Az;
                    sensor_data.gx_dps =  imuData.Gx * 180.0 / M_PI;
                    sensor_data.gy_dps = -imuData.Gy * 180.0 / M_PI;
                    sensor_data.gz_dps = -imuData.Gz * 180.0 / M_PI;
                    sensor_data.roll   =  imuData.Roll * 180.0 / M_PI;
                    sensor_data.pitch  = -imuData.Pitch * 180.0 / M_PI;
                    sensor_data.yaw    = -imuData.Yaw * 180.0 / M_PI;

                    if (protocol_matched_type == PROTOCOL_A1) {
                        sensor_data.mx =  imuData.Mx;
                        sensor_data.my = -imuData.My;
                        sensor_data.mz = -imuData.Mz;
                    } else {
                        sensor_data.mx = sensor_data.my = sensor_data.mz = 0.0;
                    }

                    sensor_data.temperature = imuData.Temp;
                    double curr_time_ms = measurement_time.seconds() * 1000.0;
                    sensor_data.delta_ms = curr_time_ms - last_time_ms;
                    

                    sensor_data.protocol = (protocol_matched_type == PROTOCOL_B4) ? "B4" :
                                           (protocol_matched_type == PROTOCOL_B6) ? "B6" :
                                           (protocol_matched_type == PROTOCOL_C4) ? "C4" :
                                           (protocol_matched_type == PROTOCOL_A1) ? "A1" : "UNKNOWN";

                    char hex_str[256] = {0};
                    int len = (protocol_matched_type == PROTOCOL_B4) ? SIZE_PROTOCOL_B4 :
                              (protocol_matched_type == PROTOCOL_B6) ? SIZE_PROTOCOL_B6 :
                              (protocol_matched_type == PROTOCOL_C4) ? SIZE_PROTOCOL_C4 :
                              (protocol_matched_type == PROTOCOL_A1) ? SIZE_PROTOCOL_A1 : 0;

                    for (int i = 0; i < len; ++i) {
                        char buf[6];
                        sprintf(buf, "%02X ", protcol_buf[i]);
                        strcat(hex_str, buf);
                    }
                    sensor_data.hex = std::string(hex_str);

                    imu_data_pub->publish(sensor_data);
                    last_time_ms = measurement_time.seconds() * 1000.0;
                    new_frame = 0;
                    memset(&imuData,0,sizeof(imuData));
                    
                  }
                }
              }
			  else
			{
				try
				{
					ser.setPort(port);
					ser.setBaudrate(cfg_baud_rate);
					serial::Timeout to = serial::Timeout::simpleTimeout(1000);
					ser.setTimeout(to);
					ser.open();
				}
				catch (serial::IOException &e)
				{
					RCLCPP_ERROR(node->get_logger(),
								"Unable to open serial port %s. Trying again in 5 seconds.",
								ser.getPort().c_str());
					rclcpp::sleep_for(std::chrono::seconds(5));
				}
			}

            } 
			catch (serial::IOException& e) 
            {
              RCLCPP_ERROR(node->get_logger(), "Serial IO error: %s", e.what());
              ser.close();
            }

        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
