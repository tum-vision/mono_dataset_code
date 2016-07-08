#include <opencv2/core/core.hpp>
#include "ExposureImage.h"
#include "Eigen/Core"


class UndistorterFOV
{
public:

	UndistorterFOV(const char* configFileName);
	UndistorterFOV();
	~UndistorterFOV();


	template<typename T>
	void undistort(const T* input, float* output, int nPixIn, int nPixOut) const;
	void distortCoordinates(float* in_x, float* in_y, int n);

	inline Eigen::Matrix3f getK_rect() const
	{
		return Krect;
	}
	inline Eigen::Matrix3f getK_org() const
	{
		return Korg;
	}
	inline float getOmega() const
	{
		return inputCalibration[4];
	}
	const Eigen::VectorXf getOriginalCalibration() const
	{
		Eigen::VectorXf vec(5);
		vec[0] = inputCalibration[0] * in_width;
		vec[1] = inputCalibration[1] * in_height;
		vec[2] = inputCalibration[2] * in_width - 0.5;
		vec[3] = inputCalibration[3] * in_height - 0.5;
		vec[4] = inputCalibration[4];
		return vec;
	}
	const Eigen::Vector2i getInputDims() const
	{
		return Eigen::Vector2i(in_width, in_height);
	}
	const Eigen::Vector2i getOutputDims() const
	{
		return Eigen::Vector2i(out_width, out_height);
	}
	
	bool isValid() const
	{
		return valid;
	}
private:
	Eigen::Matrix3f Krect;
	Eigen::Matrix3f Korg;
	float inputCalibration[5];
	float outputCalibration[5];
	int out_width, out_height;
	int in_width, in_height;

	float* remapX;
	float* remapY;

	bool valid;
};
