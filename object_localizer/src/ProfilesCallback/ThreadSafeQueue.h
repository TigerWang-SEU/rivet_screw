#ifndef __THREADSAFEQUEUE_H__
#define __THREADSAFEQUEUE_H__

#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class ThreadSafeQueue
{
  std::condition_variable cond;
  std::mutex mutex;
  std::queue<T> tsq;
  int maxSize;
public:

  ThreadSafeQueue() : maxSize( 3000 )
  { }

  ThreadSafeQueue(int mxsz) : maxSize(mxsz)
  { }

  void push ( T request )
  {
    std::unique_lock<std::mutex> lock(mutex);
    cond.wait(lock, [this]()
    { return !isFull(); });
    tsq.push(request);
    lock.unlock();
    cond.notify_all();
  }

  void pop ( T &request )
  {
    std::unique_lock<std::mutex> lock(mutex);
    cond.wait(lock, [this]()
    { return !isEmpty(); });
    request = tsq.front();
    tsq.pop();
    lock.unlock();
    cond.notify_all();
  }

  bool isFull() const
  {
    return tsq.size() >= maxSize;
  }

  bool isEmpty() const
  {
    return tsq.size() == 0;
  }

  int length() const
  {
    return tsq.size();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex);
    while ( !isEmpty() )
    {
      tsq.pop();
    }
    lock.unlock();
    cond.notify_all();
  }
};

#endif
