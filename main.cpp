#include <iostream>
#include <vector>
#include <string>
#include <boost/numeric/ublas/vector.hpp>
#include "GNG.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <ctime>

using namespace boost::numeric;
using namespace ng;

int main(int argc, char* argv[])
{
	std::srand(std::time(0));
	ng::GNG model(2, 0.3, 0.00003, 0.5, 0.0005, 20, 12, 120, 1.82, true);
	//ng::GNG model(2, 0.3, 0.00003, 0.5, 0.0005, 20, 12, 120, 3.2, true);

	ublas::vector<double> a(2);
	ublas::vector<double> x1(2), x2(2);
	x1(0) = 300 + std::rand()%40;
	x1(1) = 220 + std::rand()%40;
	x2(0) = 300 + std::rand()%40;
	x2(1) = 220 + std::rand()%40;
	model.init(x1, x2);

	int sign = 1;
	for(int i = 0; i < 2500; i++)
	{
		std::cout<<i<<"\n";
		//a(0) = 200 + std::rand()%200;
		//a(1) = 200 + std::rand()%200;

		int rnd = std::rand()%3;
		if(rnd == 0)
		{
			a(0) = 300 + std::rand()%40;
			a(1) = 220 + std::rand()%40;
		}
		else if(rnd == 1)
		{
			a(0) = 420 + std::rand()%40;
			a(1) = 290 + std::rand()%40;
			//a(0) = 420 + std::rand()%40;
			//a(1) = 290 + std::rand()%40;
		}
		else
		{
			a(0) = 100 + std::rand()%40;
			a(1) = 120 + sign*std::sqrt(400-(a(0)-120)*(a(0)-120));
			sign *= -1;
		}
		model.addSignal(a);
		//cv::Mat img(480, 640, CV_32FC3);
		//cv::rectangle(img, cv::Point(300, 220), cv::Point(340, 260), cv::Scalar(0, 255, 0));
		//cv::rectangle(img, cv::Point(420, 290), cv::Point(460, 330), cv::Scalar(0, 255, 0));
		//cv::circle(img, cv::Point(120,120), 20, cv::Scalar(0, 255, 0));
		//cv::circle(img, cv::Point(a(0), a(1)), 2, cv::Scalar(255, 0, 0), 2);
		//model.draw(img);
		//cv::imshow("img", img);
		//cv::waitKey(10);
	}
	model.classify();
	std::cout<<"graph_connected_component_count: "<<model.getConnectedComponentsCount()<<"\n";
	cv::Mat img(480, 640, CV_32FC3);
	//cv::rectangle(img, cv::Point(200, 200), cv::Point(400, 400), cv::Scalar(0, 255, 0));

	cv::rectangle(img, cv::Point(300, 220), cv::Point(340, 260), cv::Scalar(0, 255, 0));
	cv::rectangle(img, cv::Point(420, 290), cv::Point(460, 330), cv::Scalar(0, 255, 0));
	cv::circle(img, cv::Point(120,120), 20, cv::Scalar(0, 255, 0));
	model.draw(img);
	cv::imshow("img", img);
	cv::waitKey(20);
	for(int i = 0; i < 2500; i++)
	{
		std::cout<<i<<"\n";
		//a(0) = 200 + std::rand()%200;
		//a(1) = 200 + std::rand()%200;

		int rnd = std::rand()%3;
		if(rnd == 0)
		{
			a(0) = 300 + std::rand()%40;
			a(1) = 220 + std::rand()%40;
		}
		else if(rnd == 1)
		{
			a(0) = 420 + std::rand()%40;
			a(1) = 290 + std::rand()%40;
			//a(0) = 420 + std::rand()%40;
			//a(1) = 290 + std::rand()%40;
		}
		else
		{
			a(0) = 100 + std::rand()%40;
			a(1) = 120 + sign*std::sqrt(400-(a(0)-120)*(a(0)-120));
			sign *= -1;
		}
		model.addSignal(a);
		//cv::Mat img(480, 640, CV_32FC3);
		//cv::rectangle(img, cv::Point(300, 220), cv::Point(340, 260), cv::Scalar(0, 255, 0));
		//cv::rectangle(img, cv::Point(420, 290), cv::Point(460, 330), cv::Scalar(0, 255, 0));
		//cv::circle(img, cv::Point(120,120), 20, cv::Scalar(0, 255, 0));
		//cv::circle(img, cv::Point(a(0), a(1)), 2, cv::Scalar(255, 0, 0), 2);
		//model.draw(img);
		//cv::imshow("img", img);
		//cv::waitKey(10);
	}
	model.classify();
	std::cout<<"graph_connected_component_count: "<<model.getConnectedComponentsCount()<<"\n";
	//cv::rectangle(img, cv::Point(200, 200), cv::Point(400, 400), cv::Scalar(0, 255, 0));

	cv::rectangle(img, cv::Point(300, 220), cv::Point(340, 260), cv::Scalar(0, 255, 0));
	cv::rectangle(img, cv::Point(420, 290), cv::Point(460, 330), cv::Scalar(0, 255, 0));
	cv::circle(img, cv::Point(120,120), 20, cv::Scalar(0, 255, 0));
	model.draw(img);
	cv::imshow("img", img);


	cv::waitKey();

	return 0;
}