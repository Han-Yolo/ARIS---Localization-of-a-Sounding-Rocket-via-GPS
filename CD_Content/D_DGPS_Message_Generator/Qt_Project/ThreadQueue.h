/*
 * ThreadQueue.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Simon Herzog
 */

#ifndef THREADQUEUE_H_
#define THREADQUEUE_H_


#include <mutex>
#include <queue>


template<class T>
class ThreadQueue
{
private:

	std::queue<T> q;
	std::mutex m;

public:

	ThreadQueue(void) {}

	bool empty(void) { return q.empty(); }

	void push(T item)
	{
		std::lock_guard<std::mutex> lock(m);
		q.push(item);
	}

	T pop(void)
	{
		std::lock_guard<std::mutex> lock(m);
		if(q.empty()) return T();
		T item = q.front();
		q.pop();
		return item;
	}

};


#endif /* THREADQUEUE_H_ */
