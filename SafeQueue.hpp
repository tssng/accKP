#ifndef SAFEQUEUE_HPP
#define SAFEQUEUE_HPP

#include <boost/smart_ptr/make_shared_array.hpp>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "graph_types.hpp"


class SafeQueue
{
	std::queue<DataPacket> data_buffer; // maybe use a vector for better cache util
	std::mutex mtx;
	std::condition_variable cv;
public:
	void push_to_buffer(const DataPacket& data) {
		std::scoped_lock<std::mutex> lck(mtx);
		data_buffer.push(data);
		cv.notify_one();
	}
	void pop_from_buffer(DataPacket& data) {
		std::unique_lock<std::mutex> lck(mtx);
		cv.wait(lck, [this]{return !data_buffer.empty();});
		data = data_buffer.front();
		data_buffer.pop();
	}
};

#endif //SAFEQUEUE_HPP
