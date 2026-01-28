#ifndef SENSORPARSER_HPP
#define SENSORPARSER_HPP

#include "graph_types.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

/*
 * Reads measurement + odometry files and (attempts) to send packets in order in a "zipper" fashion
 */

class SensorParser
{
private:
	std::ifstream odom_file;
	std::ifstream meas_file;
	// whether there is new odometry/measurement data
	bool has_next_odom = false;
	bool has_next_meas = false;

	// holds next packet (may expand if further buffering is needed)
	DataPacket odom_buffer;
	DataPacket meas_buffer;
public:
	SensorParser(const std::string& odom_path, const std::string& meas_path) {
		odom_file.open(odom_path);
		meas_file.open(meas_path);

		getNextOdom();
		getNextMeas();
	}
	~SensorParser() = default;
	// online parser
	DataPacket getNextPacket() {
		// if both buffers are full, return the packet with the oldest associated timestamp
		if (has_next_odom && has_next_meas)
		{
			if (odom_buffer.timestamp < meas_buffer.timestamp)
			{
				const DataPacket to_return = odom_buffer;
				getNextOdom();
				return to_return;
			} else
			{
				const DataPacket to_return = meas_buffer;
				getNextMeas();
				return to_return;
			}
		}
		// check if either buffer is still full
		if (has_next_odom)
		{
			const DataPacket to_return = odom_buffer;
			getNextOdom();
			return to_return;
		}
		if (has_next_meas)
		{
			const DataPacket to_return = meas_buffer;
			getNextMeas();
			return to_return;
		}

		// no more sensor data available
		DataPacket to_return;
		to_return.type = END_OF_LOG;
		return to_return;
	}
	// ignore - used for different dataset (without GTSAM)
	static PoseGraph loadGraphG2O(const std::string& filename)
	{
		PoseGraph poseGraph;
		std::ifstream inputFile(filename);
		if (!inputFile)
		{
			std::cerr << "Error opening file " << filename << std::endl;
		}
		std::string line;
		int i = 0;
		while (std::getline(inputFile, line))
		{
			std::istringstream iss(line);
			std::string type;

			iss >> type;
			if (type == "VERTEX_SE2")
			{
				Vertex vertex;
				iss >> vertex;
				poseGraph.vertices.insert(std::make_pair(vertex.id, vertex));
			}
			else if (type == "EDGE_SE2")
			{
				Edge edge;
				iss >> edge;
				poseGraph.edges.push_back(edge);
			}
			else
			{
				std::cerr << "Error identifying type " << type << std::endl;
			}
			++i;
		}
		std::cout << "Done reading after " << i << " lines!" << std::endl;
		return poseGraph;
	}

private:
	DataPacket parseLine(const std::string& line, DataType type) {
		std::stringstream ss(line);
		DataPacket data;
		if (type == ODOMETRY)
		{
			data.type = ODOMETRY;
			ss >> data.timestamp >> data.f_velocity >> data.a_velocity;
		}
		else if (type == MEASUREMENT)
		{
			data.type = MEASUREMENT;
			ss >> data.timestamp >> data.subject >> data.range >> data.bearing;
		}
		else // should not ever reach here
		{
			data.type = END_OF_LOG;
		}
		return data;
	}
	// helpers that retrieve packets for each file type + update buffer/flags
	void getNextOdom() {
		std::string line;
		if (std::getline(odom_file, line))
		{
			odom_buffer = parseLine(line, ODOMETRY);
			// indicate that buffer has data
			has_next_odom = true;
		}
		else
		{
			has_next_odom = false;
		}
	}
	void getNextMeas() {
		std::string line;
		if (std::getline(meas_file, line))
		{
			meas_buffer = parseLine(line, MEASUREMENT);
			has_next_meas = true;
		}
		else
		{
			has_next_meas = false;
		}
	}
};

#endif
