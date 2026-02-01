#ifndef SINGLESOLVER_HPP
#define SINGLESOLVER_HPP

#include "graph_types.hpp"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
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
#include <map>

struct DualState {
    gtsam::Pose2 neighbor_estimate;
    double y = 0.0;
    double theta = 0.0;
    bool initialized = false;
};

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
    // neighbours
    std::unordered_set<int> robot_keys {5,14,41,32,23};
    // map from separator -> dual vars & neighbour info
    std::map<gtsam::Symbol, DualState> separator_states;

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

    gtsam::noiseModel::Diagonal::shared_ptr consensus_noise;

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

        odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.02));
        prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.01));
        br_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.05));
        // consensus_noise = how much we trust our neighbours
        consensus_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
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

        network_thread = std::thread(&SingleSolver::networkLoop, this);
        sensor_thread = std::thread(&SingleSolver::sensorLoop, this);
        solverLoop();
    }

    void saveResults(const std::string& filename) const {
        gtsam::Values current_estimate = isam.calculateEstimate();
        std::cout << "Saving " << current_estimate.size() << " variables to " << filename << std::endl;

        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open " << filename << " for writing." << std::endl;
            return;
        }

        file << std::fixed << std::setprecision(6);

        for (const auto& key_value : current_estimate) {
            gtsam::Symbol key(key_value.key);
            if (key.chr() == id) {
                try {
                    auto pose = current_estimate.at<gtsam::Pose2>(key);
                    file << key_to_timestamp.at(key) << " "
                         << pose.x() << " "
                         << pose.y() << " "
                         << pose.theta() << "\n";
                } catch(gtsam::ValuesIncorrectType& e) { }
            }
        }
        file.close();
    }


private:
    // constants
    double LINEAR_THRESHOLD {0.1};
    double ANGULAR_THRESHOLD {0.15};

    std::array<uint8_t, 1024> recv_buffer {0};
    udp::endpoint rx_endpoint;

    void addNewPose() {
        const gtsam::Symbol cur_key(id, pose_index);
        pose_index++;
        const gtsam::Symbol next_key(id, pose_index);

        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            cur_key, next_key, odom_accum, odom_noise)
            );

        const gtsam::Pose2 next_pose = current_pose.compose(odom_accum);
        new_estimates.insert(next_key, next_pose);
        odom_accum = gtsam::Pose2::Identity();

        current_pose = next_pose;
        key_to_timestamp[next_key] = last_timestamp;
    }

    // constantly reads from file
    void sensorLoop() {
        while (running)
        {
            DataPacket data = parser.getNextPacket();
            if (data.type == END_OF_LOG)
            {
                data_queue.push_to_buffer(data);
                break;
            }
            data_queue.push_to_buffer(data);
        }
    }


    void networkLoop() {
        start_async_receive();
        io_context.run();
    }

    void start_async_receive() {
        rx_socket.async_receive_from(
            boost::asio::buffer(recv_buffer), rx_endpoint,
            [this](const boost::system::error_code& ec, std::size_t len) {
                if (!ec && len > 0) {
                    const auto fb_packet = remoteData::GetDataPacket(recv_buffer.data());

                    DataPacket packet;

                    packet.type = MEASUREMENT;
                    packet.timestamp = fb_packet->timestamp();

                    // f_velocity/a_velocity mapped to X/Y for separator pose
                    packet.f_velocity = fb_packet->f_velocity();
                    packet.a_velocity = fb_packet->a_velocity();
                    // bearing/Range mapped to dual vars
                    packet.dual_var_y = fb_packet->dual_var_y();
                    packet.dual_var_theta = fb_packet->dual_var_theta();

                    packet.subject = fb_packet->subject(); // id of the robot sending this
                    packet.bearing = fb_packet->bearing(); // theta of separator

                    packet.is_separ = true;

                    data_queue.push_to_buffer(packet);
                    start_async_receive();
                }
            });
    }

    void sendSeparatorPacket(int target_id, const gtsam::Pose2& pose, double dy, double dth) {
        flatbuffers::FlatBufferBuilder builder(1024);

        // pack the estimate of the shared node
        //reuse fields: f_vel -> x, a_vel -> y, bearing -> theta
        auto packet_offset = remoteData::CreateDataPacket(
                builder,
                remoteData::DataType_MEASUREMENT,
                last_timestamp,
                pose.x(),           // f_velocity field used for X
                pose.y(),           // a_velocity field used for Y
                id,                 // subject = MY ID (sender)
                0.0,                // range unused
                pose.theta(),       // bearing field used for Theta
                dy,                 // dual var y
                dth                 // dual var theta
        );
        builder.Finish(packet_offset);

		udp::endpoint dest(
            boost::asio::ip::make_address("127.0.0.1"),
            5000 + target_id
        );

    	//fire/forget
        tx_socket.send_to(boost::asio::buffer(builder.GetBufferPointer(), builder.GetSize()), dest);
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

    bool processData(const DataPacket& data) {
        if (data.type == END_OF_LOG) return false;

        if (!initialized)
        {
            const gtsam::Symbol init_key(id, pose_index);
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(init_key, current_pose, prior_noise));
            new_estimates.insert(init_key, current_pose);
            isam.update(graph, new_estimates);

            graph.resize(0);
            new_estimates.clear();
            initialized = true;
            last_timestamp = data.timestamp;
            key_to_timestamp[init_key] = last_timestamp;
            return true;
        }

        // separator update from network
        if (data.is_separ)
        {
            // 'subject' in packet is the Sender's ID.

            int neighbor_id = data.subject;
            gtsam::Symbol sep_key('L', neighbor_id);

            // update Dual State
            separator_states[sep_key].neighbor_estimate = gtsam::Pose2(data.f_velocity, data.a_velocity, data.bearing);
            separator_states[sep_key].y = data.dual_var_y;
            separator_states[sep_key].theta = data.dual_var_theta;
            separator_states[sep_key].initialized = true;

            // iMESA Step - add a PriorFactor to pull my estimate of L_X towards
            // (neighbor estimate + dual vars)
            if (isam.valueExists(sep_key)) {
                 graph.add(gtsam::PriorFactor<gtsam::Pose2>(
                     sep_key,
                     separator_states[sep_key].neighbor_estimate,
                     consensus_noise
                 ));

                 // trigger update to propagate consensus
                 isam.update(graph, new_estimates);
                 graph.resize(0);
                 new_estimates.clear();
            }
            return true;
        }

        // 3. Local Odometry
        if (data.type == ODOMETRY)
        {
            const double dt = data.timestamp - last_timestamp;
            if (dt <= 0) return true;

            gtsam::Vector3 odom_vector(data.f_velocity, 0.0, data.a_velocity);
            odom_vector = odom_vector * dt; // integration

            last_timestamp = data.timestamp;

            odom_accum = odom_accum.compose(gtsam::Pose2::Expmap(odom_vector));
            current_pose = current_pose.compose(gtsam::Pose2::Expmap(odom_vector));

            if (odom_accum.translation().norm() > LINEAR_THRESHOLD ||
                std::abs(odom_accum.theta()) > ANGULAR_THRESHOLD)
            {
                addNewPose();
            }
        }

        if (data.type == MEASUREMENT)
        {
            // clean up odom buffer before adding measurement
            if (odom_accum.translation().norm() > 0.001 || std::abs(odom_accum.theta()) > 0.001) {
                addNewPose();
            }

            gtsam::Symbol cur_key(id, pose_index);
            gtsam::Symbol landmark_key('L', data.subject);

            // check if this is robot-robot/a separator node
            if (robot_keys.contains(data.subject))
            {
                if (!isam.valueExists(landmark_key)) {
                    // initialize based on measurement
                	double cx = data.range * std::cos(data.bearing);
                	double cy = data.range * std::sin(data.bearing);

                	// create the relative pose
                	gtsam::Pose2 relative_pose(cx, cy, 0.0);
                	gtsam::Pose2 init_est = current_pose.compose(relative_pose);

                	new_estimates.insert(landmark_key, init_est);
                }
                graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>(
                    cur_key, landmark_key, gtsam::Rot2(data.bearing), data.range, br_noise));

                // update ISAM
                isam.update(graph, new_estimates);
                graph.resize(0);
                new_estimates.clear();

                // get our current belief of the separator
                gtsam::Pose2 sep_est = isam.calculateEstimate<gtsam::Pose2>(landmark_key);

                // prepare Dual Vars - need to update based on error
                double dy = separator_states[landmark_key].y;
                double dth = separator_states[landmark_key].theta;

                // send to Neighbor
                sendSeparatorPacket(data.subject, sep_est, dy, dth);
            }
            else
            {
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
        }
        return true;
    }
};

#endif