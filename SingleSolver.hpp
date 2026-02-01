#ifndef SINGLESOLVER_HPP
#define SINGLESOLVER_HPP

#include "graph_types.hpp"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "SafeQueue.hpp"
#include "SensorParser.hpp"
#include "packet_generated.h"
#include <boost/asio.hpp>

using boost::asio::ip::udp;

//testing
#include <fstream>
#include <iomanip>
#include <unordered_set>
#include <atomic>


class SingleSolver
{
private:
	int id;
	gtsam::ISAM2 isam;
	SensorParser parser;

	// thread handling
	std::atomic<bool> running{false};
	std::thread sensor_thread,
				network_thread;
	SafeQueue data_queue;
	boost::asio::io_context io_context;
	udp::socket tx_socket,
				rx_socket;

	//for testing purposes
	std::map<gtsam::Symbol, double> key_to_timestamp;
	std::unordered_set<int> robot_keys {5,14,41,32,23};

	// "buffer" vars
	gtsam::NonlinearFactorGraph graph;
	gtsam::Values new_estimates;
	gtsam::Pose2 current_pose;
	gtsam::Pose2 odom_accum;
	double last_timestamp {0.0};
	bool initialized {false};

	int pose_index {0};

	// define noise models
	gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
	gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
	gtsam::noiseModel::Diagonal::shared_ptr br_noise;
public:
	explicit SingleSolver(const int id,
						const double x,
						const double y,
						const double theta,
						const std::string& odom_path,
						const std::string& meas_path) :
			id(id),
			parser(odom_path, meas_path),
			io_context(),
			tx_socket(io_context, udp::endpoint(udp::v4(), 0)),
			rx_socket(io_context, udp::endpoint(udp::v4(), 5000 + id)) {

		graph = gtsam::NonlinearFactorGraph();
		odom_accum = gtsam::Pose2();
		current_pose = gtsam::Pose2(x, y, theta);

		odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
		prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
		br_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1));

	}

	~SingleSolver() {
		running = false;
		io_context.stop();

		if (tx_socket.is_open()) {tx_socket.close();}
		if (rx_socket.is_open()) {rx_socket.close();}
		if (sensor_thread.joinable()) {sensor_thread.join();}
		if (network_thread.joinable()) {network_thread.join();}
	}

	void run() {
		if (running) {return;}
		running = true;

		sensor_thread = std::thread(&SingleSolver::sensorLoop, this);
		//network_thread = std::thread(&SingleSolver::networkLoop, this);

		solverLoop();
	}

	void saveResults(const std::string& filename) const {
		gtsam::Values current_estimate = isam.calculateEstimate();
		std::cout << current_estimate.size() << std::endl;

		std::ofstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error: Could not open " << filename << " for writing." << std::endl;
			return;
		}

		// set formatting for precision
		file << std::fixed << std::setprecision(6);

		// iterate over all variables in the graph
		for (const auto& key_value : current_estimate) {
			gtsam::Symbol key(key_value.key);

			// robot poses
			if (key.chr() == id) { // valid robot chars
				try {
					auto pose = current_estimate.at<gtsam::Pose2>(key);

					// format: [Type] [Timestamp] [X] [Y] [Theta]
					file << key_to_timestamp.at(key) << " "
						 << pose.x() << " "
						 << pose.y() << " "
						 << pose.theta() << "\n";
				} catch(gtsam::ValuesIncorrectType& e) {
					// ignore if not a pose2
				}
			}

			/*
			// landmarks
			else if (key.chr() != id) {
				try {
					auto pt = current_estimate.at<gtsam::Point2>(key);

					// Format: [Type] [ID] [X] [Y]
					file << "LANDMARK " << key.index() << " "
						 << pt.x() << " "
						 << pt.y() << "\n";
				} catch(gtsam::ValuesIncorrectType& e) {
					// Ignore
				}
			}
			*/
		}

		file.close();
		std::cout << "Results saved to " << filename << std::endl;
	}


private:
	// constants for downsampling
	double LINEAR_THRESHOLD {0.05};
	double ANGULAR_THRESHOLD {0.1};

	std::array<uint8_t, 1024> recv_buffer {0};
	udp::endpoint rx_endpoint;

	void addNewPose() {
		const gtsam::Symbol cur_key(id, pose_index);
		pose_index++;
		const gtsam::Symbol next_key(id, pose_index);
		graph.add(gtsam::BetweenFactor(
			cur_key, next_key, odom_accum, odom_noise)
			);

		const gtsam::Pose2 next_pose = current_pose.compose(odom_accum);
		new_estimates.insert(next_key, next_pose);
		odom_accum = gtsam::Pose2::Identity();

		current_pose = next_pose;
		key_to_timestamp[next_key] = last_timestamp;
	}

	void sensorLoop() {
		while (running)
		{
			DataPacket data = parser.getNextPacket();
			if (data.type == END_OF_LOG) // no more data
			{
				data_queue.push_to_buffer(data);
				break;
			}
			/*
			if (robot_keys.contains(data.subject)) // received a separator node
			{
				// need to send to relevant agent

				// build flatbuffer
				flatbuffers::FlatBufferBuilder builder(1024);
				auto packet_offset = remoteData::CreateDataPacket(
						builder,
						remoteData::DataType_MEASUREMENT, // type
						data.timestamp,      // timestamp
						data.f_velocity,               // f_velocity
						data.a_velocity,               // a_velocity
						data.subject,                 // subject (unused for odom)
						data.range,               // range (unused for odom)
						data.bearing,                // bearing (unused for odom)
						true
	);
				builder.Finish(packet_offset);

				// get endpoint and send
				boost::asio::ip::address addr = boost::asio::ip::make_address("127.0.0.1");
				udp::endpoint dest(addr, 5000 + data.subject);

				tx_socket.send_to(boost::asio::buffer(builder.GetBufferPointer(), builder.GetSize()), dest);
				continue;
			}
			*/
			data_queue.push_to_buffer(data);

		}
	}


	void networkLoop() {
		start_async_receive();
		io_context.run();
	}


	void solverLoop() {
		DataPacket data;
		while (running)
		{
			data_queue.pop_from_buffer(data);
			if (!processData(data)) {break;}
		}

		running = false;

		if (sensor_thread.joinable()) {sensor_thread.join();}
		if (network_thread.joinable()) {network_thread.join();}
	}

	void start_async_receive() {
		rx_socket.async_receive_from(
			boost::asio::buffer(recv_buffer), rx_endpoint,
			[this](const boost::system::error_code& ec, std::size_t len) {
				if (!ec && len > 0) {
					const auto fb_packet = remoteData::GetDataPacket(recv_buffer.data());

					DataPacket packet;
					packet.type = static_cast<DataType>(fb_packet->type());
					packet.timestamp = fb_packet->timestamp();

					// estimated global frame vars
					packet.f_velocity = fb_packet->f_velocity();
					packet.a_velocity = fb_packet->a_velocity();
					packet.range = fb_packet->range();

					packet.subject = fb_packet->subject();

					// dual variable updates
					packet.bearing = fb_packet->bearing();
					packet.dual_var_y = fb_packet->dual_var_y();
					packet.dual_var_theta = fb_packet->dual_var_theta();

					packet.is_separ = true; //mark as separator
					data_queue.push_to_buffer(packet);
					start_async_receive(); // loop the async call
				}
			});
	}

	bool processData(const DataPacket& data) {
		if (data.type == END_OF_LOG) return false;
		if (!initialized)
		{
			// set the prior pose to the origin and set timestamp to start of data
			const gtsam::Symbol init_key(id, pose_index);

			graph.add(gtsam::PriorFactor(init_key, current_pose, prior_noise));
			new_estimates.insert(init_key, current_pose);

			isam.update(graph, new_estimates);

			graph.resize(0);
			new_estimates.clear();

			initialized = true;
			last_timestamp = data.timestamp;

			key_to_timestamp[init_key] = last_timestamp;
			return true;
		}
		if (data.type == ODOMETRY)
		{
			const double dt {data.timestamp - last_timestamp};
			if (dt < 0) {return true;} // will need to handle out-of-order issues later
			// create twist vector
			gtsam::Vector3 odom_vector(data.f_velocity, 0.0, data.a_velocity);
			// convert to "step" in tangent space
			odom_vector = odom_vector * (dt);
			last_timestamp = data.timestamp;
			// add movement to transformatino accumulator
			odom_accum = odom_accum.compose(gtsam::Pose2::Expmap(odom_vector));
			current_pose = current_pose.compose(gtsam::Pose2::Expmap(odom_vector));


			// if robot has moved / turned enough, then update pose
			const double trans {odom_accum.translation().norm()};
			const double rot {std::abs(odom_accum.theta())};
			if (trans > LINEAR_THRESHOLD || rot > ANGULAR_THRESHOLD)
			{
				addNewPose();
				// don't trigger optimization yet - unless we want odometry only chains (not the worst thing ever)
			}

		}
		if (data.type == MEASUREMENT)
		{
			if (robot_keys.contains(data.subject)) // separator node
			{
				return true;
			}
			// clear out accumulated odometry "buffer" to ensure that landmark is positioned correctly in global space
			if (odom_accum.translation().norm() > 0.01 || std::abs(odom_accum.theta()) > 0.01)
			{
				addNewPose();
			}

			const gtsam::Symbol landmark_key('L', data.subject);
			const gtsam::Symbol cur_key(id, pose_index);
			// plan: check if landmark already exists in ISAM (?)
				// if it does, then add a bearing range factor using current pose as the other node
				// otherwise, add a new node then link to current pose
				// may need to check timestamps... except we are always receiving the oldest timestamp so not sure
			if (!isam.valueExists(landmark_key))
			{
				const double cart_x {data.range * std::cos(data.bearing)};
				const double cart_y {data.range * std::sin(data.bearing)};

				const gtsam::Point2 l_estimate {current_pose.transformFrom(gtsam::Point2(cart_x, cart_y))};
				new_estimates.insert(landmark_key, l_estimate);
			}

			graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
				cur_key, landmark_key, data.bearing, data.range, br_noise));


			isam.update(graph, new_estimates);

			// sync local tracker with the optimizer's belief
			current_pose = isam.calculateEstimate<gtsam::Pose2>(cur_key);

			graph.resize(0);
			new_estimates.clear();
		}
		return true;
	}

};

#endif