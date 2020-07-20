#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
	float dx() const {return x_max - x_min;}
	float dy() const {return y_max - y_min;}
	float dz() const {return z_max - z_min;}
	float vol() const {return dx() * dy() * dz();}
};
#endif