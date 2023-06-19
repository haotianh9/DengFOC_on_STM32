/*
 * lowpass_filter.h
 *
 *  Created on: Jun 18, 2023
 *      Author: hht
 */

#ifndef INC_LOWPASS_FILTER_H_
#define INC_LOWPASS_FILTER_H_

struct LowPassFilter{
	float Tf; //!< 低通滤波时间常数
//	unsigned long timestamp_prev;  //!< 最后执行时间戳
	float y_prev; //!< 上一个循环中的过滤后的值
};
float LowPassFilter_operator(float x, struct LowPassFilter* filter);
#endif /* INC_LOWPASS_FILTER_H_ */
