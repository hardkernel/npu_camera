#include "camera.h"

using namespace std::chrono_literals;

void CamPublisher_::initialize()
{
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
	pub_ = this->create_publisher<sensor_msgs::msg::Image>(
			"/camera/mat2image_image2mat", qos);

	cap.open(0);

	if (!cap.isOpened()) {
		RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
		throw std::runtime_error("Could not open video stream");
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);

	timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(1000 / 30)), // 30 fps
			std::bind(&CamPublisher_::timerCallback, this));
}

void CamPublisher_::init_rga()
{
	memset(&src_rect, 0, sizeof(src_rect));
	memset(&dst_rect, 0, sizeof(dst_rect));
	memset(&src, 0, sizeof(src));
	memset(&dst, 0, sizeof(dst));
}

int CamPublisher_::create_network()
{
	printf("Loading mode...\n");
	model_data_size = 0;
	model_data = load_model(model_name, &model_data_size);
	ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
	if (ret < 0)
	{
		printf("rknn_init error ret=%d\n", ret);
		return -1;
	}

	ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version,
			sizeof(rknn_sdk_version));
	if (ret < 0)
	{
		printf("rknn_init error ret=%d\n", ret);
		return -1;
	}
	printf("sdk version: %s driver version: %s\n", version.api_version,
			version.drv_version);

	ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
	if (ret < 0)
	{
		printf("rknn_init error ret=%d\n", ret);
		return -1;
	}
	printf("model input num: %d, output num: %d\n", io_num.n_input,
			io_num.n_output);

	memset(input_attrs, 0, sizeof(input_attrs));
	for (uint32_t i = 0; i < io_num.n_input; i++)
	{
		input_attrs[i].index = i;
		ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]),
				sizeof(rknn_tensor_attr));
		if (ret < 0)
		{
			printf("rknn_init error ret=%d\n", ret);
			return -1;
		}
	}

	memset(output_attrs, 0, sizeof(output_attrs));
	for (uint32_t i = 0; i < io_num.n_output; i++)
	{
		output_attrs[i].index = i;
		ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]),
				sizeof(rknn_tensor_attr));
	}

	channel = 3;
	width = 0;
	height = 0;

	if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
	{
		channel = input_attrs[0].dims[1];
		width = input_attrs[0].dims[2];
		height = input_attrs[0].dims[3];
	}
	else
	{
		width = input_attrs[0].dims[1];
		height = input_attrs[0].dims[2];
		channel = input_attrs[0].dims[3];
	}

	memset(inputs, 0, sizeof(inputs));
	inputs[0].index = 0;
	inputs[0].type = RKNN_TENSOR_UINT8;
	inputs[0].size = width * height * channel;
	inputs[0].fmt = RKNN_TENSOR_NHWC;
	inputs[0].pass_through = 0;

	resize_buf = malloc(height * width * channel);

	return 0;
}

unsigned char * CamPublisher_::load_data(FILE *fp, size_t ofst, size_t sz)
{
	unsigned char *data;
	int ret;

	data = NULL;

	if (NULL == fp)
		return NULL;

	ret = fseek(fp, ofst, SEEK_SET);

	if (ret != 0) {
		printf("blob seek failure.\n");
		return NULL;
	}

	data = (unsigned char *)malloc(sz);
	if (data == NULL) {
		printf("buffer malloc failure.\n");
		return NULL;
	}
	ret = fread(data, 1, sz, fp);
	return data;
}

unsigned char * CamPublisher_::load_model(const char *filename, int *model_size)
{
	FILE *fp;
	unsigned char *data;

	fp = fopen(filename, "rb");
	if (NULL == fp)
	{
		printf("Open file %s failed.\n", filename);
		return NULL;
	}

	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);

	data = load_data(fp, 0, size);

	fclose(fp);

	*model_size = size;
	return data;
}
/*
   int saveFloat(const char *file_name, float *output, int element_size)
   {
   FILE *fp;
   fp = fopen(file_name, "w");
   for (int i = 0; i < element_size; i++)
   {
   fprintf(fp, "%.6f\n", output[i]);
   }
   fclose(fp);
   return 0;

   }
   */
void CamPublisher_::timerCallback()
{
	cap >> frame;

	if (frame.empty()) {
		return;
	}

	//proccessing
	src = wrapbuffer_virtualaddr(frame.data, frame.cols, frame.rows, RK_FORMAT_RGB_888, frame.cols, frame.rows);
	dst = wrapbuffer_virtualaddr(resize_buf, width, height, RK_FORMAT_RGB_888, width, height);
	ret = imcheck(src, dst, src_rect, dst_rect, 0);
	if (IM_STATUS_NOERROR != ret) {
		exit(0);
	}

	convert_and_publish(frame);
}

std::string CamPublisher_::mat2encoding(int mat_type)
{
	switch (mat_type) {
		case CV_8UC1:
			return "mono8";
		case CV_8UC3:
			return "bgr8";
		case CV_16SC1:
			return "mono16";
		case CV_8UC4:
			return "rgba8";
		default:
			std::runtime_error("Unsupported encoding type");

	}

	return 0;
}

void CamPublisher_::convert_and_publish(const cv::Mat& frame)
{
	auto msg = sensor_msgs::msg::Image();

	msg.height = frame.rows;
	msg.width = frame.cols;
	msg.encoding = mat2encoding(std::move(frame.type()));
	msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);

	size_t size = frame.step * frame.rows;
	msg.data.resize(size);
	memcpy(&msg.data[0], frame.data, size);

	pub_->publish(msg);
}
