

class ExposureImage
{
public:
	float* image;
	double timestamp;
	int w,h;
	float exposure_time;
	int id;

	inline ExposureImage(int w_, int h_, double timestamp_, float exposure_, int id_)
		: timestamp(timestamp_), w(w_), h(h_), exposure_time(exposure_), id(id_)
	{
		image = new float[w*h];
	}
	inline ~ExposureImage()
	{
		delete[] image;
	}
};

