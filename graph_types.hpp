#ifndef GRAPH_TYPES_HPP
#define GRAPH_TYPES_HPP

#include <Eigen/Core>
#include <vector>
#include <unordered_map>

struct Vertex
{
	int id;
	Eigen::Vector3d pos;

	friend std::istream& operator>>(std::istream& is, Vertex& v)
	{
		return is >> v.id >> v.pos[0] >> v.pos[1] >> v.pos[2];
	}

	friend std::ostream& operator<<(std::ostream& os, const Vertex& v)
	{
		os << "Vertex ID: " << v.id << " Pos: " << v.pos.transpose();
		return os;
	}
};

struct Edge
{
	int id1, id2;
	Eigen::Vector3d measure;
	Eigen::Matrix3d info;
	friend std::istream& operator>>(std::istream& is, Edge& e)
	{
		is >> e.id1 >> e.id2 >> e.measure[0] >> e.measure[1] >> e.measure[2];

		double i11, i12, i13, i22, i23, i33;
		// Load in Upper Triangular region
		is >> i11 >> i12 >> i13 >> i22 >> i23 >> i33;

		// Matrix is symmetric
		e.info(0) = i11;
		e.info(1) = i12;
		e.info(2) = i13;

		e.info(3) = i12;
		e.info(4) = i22;
		e.info(5) = i23;

		e.info(6) = i13;
		e.info(7) = i23;
		e.info(8) = i33;

		return is;
	}

	friend std::ostream& operator<<(std::ostream& os, const Edge& e)
	{
		os << "Edge: " << e.id1 << " -> " << e.id2 << "\n"
		   << "Measure: " << e.measure.transpose() << "\n"
		   << "Info Matrix:\n" << e.info << "\n"
		   << "----------------------------" << "\n";
		return os;
	}
};

struct PoseGraph
{
	std::unordered_map<int, Vertex> vertices;
	std::vector<Edge> edges;

	// In case sizes are known
	void reserve(const int v_size, const int e_size) {
		vertices.reserve(v_size);
		edges.reserve(e_size);
	}
};

enum DataType { ODOMETRY, MEASUREMENT, END_OF_LOG};
struct DataPacket
{
	DataType type = END_OF_LOG;
	double timestamp = 0.0;
	// type == ODOMETRY
	double f_velocity = 0.0,
	       a_velocity = 0.0;
	// type == MEASUREMENT
	int subject = 0;
	double range = 0.0,
		   bearing = 0.0,
	       dual_var_y = 0.0,
	       dual_var_theta = 0.0;
	bool is_separ = false;
};

#endif //GRAPH_TYPES_HPP
