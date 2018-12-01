/*
 * LidarDataResponse.h
 *
 *  Created on: Nov 12, 2018
 *      Author: Krzysiek
 */

#ifndef LIDAR_LIDARDATARESPONSE_H_
#define LIDAR_LIDARDATARESPONSE_H_

enum LidarPosition
{
	LIDAR_POSITION_FRONT = 0,
	LIDAR_POSITION_FRONT_RIGHT,
	LIDAR_POSITION_RIGHT,
	LIDAR_POSITION_BACK_RIGHT,
	LIDAR_POSITION_BACK,
	LIDAR_POSITION_BACK_LEFT,
	LIDAR_POSITION_LEFT,
	LIDAR_POSITION_FRONT_LEFT
};

struct LidarDataResponse
{
	int quality;
	float angle;
	float distance;

	LidarDataResponse()
	{
		quality = 0;
		angle = 0;
		distance = 0;
	}

	LidarDataResponse(int p_quality, float p_angle, float p_distance)
	{
		quality = p_quality;
		angle = p_angle;
		distance = p_distance;
	}

	LidarDataResponse(char* p_responseBuffer)
	{
		quality = p_responseBuffer[0] >> 2;
		angle = ((p_responseBuffer[2] << 8 | p_responseBuffer[1]) >> 1) >> 6;
		distance = (p_responseBuffer[4] << 8 | p_responseBuffer[3]) >> 2;
	}
};

#endif /* LIDAR_LIDARDATARESPONSE_H_ */
