#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <time.h>
#include <assert.h>

#include "../inc/hypstar.h"

using namespace std;

int main() {
	std::string port = "/dev/ttyUSB1";
	port = "/dev/ttyUSB0";
	Hypstar *hs = Hypstar::getInstance(port);
	if (!hs) {
		printf("--------------\nC++ Test fail\n");
		return 1;
	}
//	hs->setLoglevel(DEBUG);
//	hs->setLoglevel(TRACE);
//	hs->setBaudRate(B_115200);
//	hs->setBaudRate(B_460800);
//	hs->setBaudRate(B_921600);
//	hs->setBaudRate(B_3000000);
//	hs->setBaudRate(B_6000000);
	hs->setBaudRate(B_8000000);

	s_img_data_holder *target_image = (s_img_data_holder*)malloc(sizeof(s_img_data_holder));
	char filename[40] = "image.jpeg";
	struct tm *timenow;
	time_t now = time(NULL);
	timenow = gmtime(&now);
	unsigned int image_size = 0;
//
//	auto t1 = std::chrono::high_resolution_clock::now();
//	unsigned int image_size = hs->acquireJpegImage(false, true, true, target_image);
//	auto t2 = std::chrono::high_resolution_clock::now();
//
//	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
//	float throughput = (float)image_size/((float)duration*1000)*(float)1024;
//	printf("Got %lu bytes in %ld ms (%f kBps = %f kbps)\n", image_size, duration, throughput, throughput*8);
//
//	assert(image_size == target_image->image_size);
//

//
//	if (image_size)
//	{
//		// save image
//		strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.jpeg", timenow);
//		printf("Filename: %s\n", filename);
//		std::ofstream outfile (filename, std::ofstream::binary);
//		outfile.write((const char *)target_image->image_data_jpeg.image_body, target_image->image_size);
//		outfile.close();
//	}
//	else
//	{
//		printf("Failed to capture image\n");
//		delete hs;
//	}

	printf("--------------\nC++ Test pass\n");

	hypstar_t *pHs;
	pHs = hypstar_init(port.c_str());
//	hypstar_set_baudrate(pHs, B_8000000);
	// @TODO: first capture with AF on fails?
	while(1)
	{
		image_size = hypstar_capture_JPEG_image(pHs, true, false, true);
		if (image_size)
		{
			image_size = hypstar_download_JPEG_image(pHs, target_image);
		}

		// save image
		if (image_size)
		{
			now = time(NULL);
			timenow = gmtime(&now);
//			strftime(filename, sizeof(filename), "images/%Y-%m-%d_%H-%M-%S-2.jpeg", timenow);
			char *pImgAsChars = (char*)target_image->image_data_jpeg.image_body;
			char *pLocation = (char*)&pImgAsChars[target_image->image_size-5];
			uint32_t crc32 = *(uint32_t*)pLocation;
//			printf("img location p: %p, Location: %p\n", &pImgAsChars[target_image->image_size-4], pLocation);
			pLocation = pLocation -2;
			uint16_t err_cnt = *(uint8_t*)pLocation;
			pLocation = pLocation -2;
			uint8_t avg = *(uint8_t*)pLocation;
			pLocation = pLocation-2;
			uint16_t shutter = *(uint16_t*)pLocation;
			pLocation = pLocation-2;
			uint16_t gain = *(uint16_t*)pLocation;
//			char *beginning = (char*)pImgAsChars;
//			char *end = &pImgAsChars[target_image->image_size];
//			printf("Filename: %s, size: %lu, crc32: 0x%08x, beginning: %p, end: %p, size from ptrs: %lu, gain: %d shutter: %lu\n", filename, target_image->image_size, crc32, beginning, end, size, gain, shutter);
			uint8_t target_value = 54;
			float gain_ratio = ((float)target_value/avg);
			printf("Filename: %s, size: %lu, crc32: 0x%08x, target: %d, gain: %d shutter: %lu, avg = %d, errs: %d, ratio: %f\n", filename, target_image->image_size, crc32, target_value, gain, shutter, avg, err_cnt, gain_ratio);
			std::ofstream outfile2(filename, std::ofstream::binary);
			outfile2.write((const char *)target_image->image_data_jpeg.image_body, target_image->image_size);
			outfile2.close();
		}
		else
		{
			printf("Failed to capture image\n");
		}
//		sleep(120);
//		sleep(2);
	}
	free(target_image);
	hypstar_close(pHs);
	printf("--------------\nC Test pass\n");
}
