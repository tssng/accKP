#include <iostream>
#include <thread>
#include "SensorParser.hpp"
#include "SingleSolver.hpp"

void testSingle() {
	std::string odom_path ="../data/Robot1_Odometry.dat";
	std::string meas_path = "../data/Robot1_Measurement.dat";
	SensorParser parser("../data/Robot1_Odometry.dat", "../data/Robot1_Measurement.dat");
	SingleSolver test_bot('a', 3.3,-3.3,2.3,odom_path, meas_path);

	test_bot.run();

	test_bot.saveResults("../resultsSingle.txt");

	std::cout << "Done!\n";
}

void testMultiple() {
	SingleSolver robotA('a', 3.3,-3.3,2.3, "../data/Robot1_Odometry.dat", "../data/Robot1_Measurement.dat");
	SingleSolver robotB('b', 0.6,-1.4,1.3, "../data/Robot2_Odometry.dat", "../data/Robot2_Measurement.dat");

	std::vector<std::thread> threads;

	threads.emplace_back([&](){ robotA.run(); });
	threads.emplace_back([&](){ robotB.run(); });

	for (auto& thread : threads)
	{
		if (thread.joinable())
		{
			thread.join();
		}
	}

	robotA.saveResults("../resultsRobotA.txt");
	robotB.saveResults("../resultsRobotB.txt");

	std::cout << "All robots finished. Simulation complete." << std::endl;
}

int main() {

	testSingle();
	//testMultiple();

	return 0;
}

