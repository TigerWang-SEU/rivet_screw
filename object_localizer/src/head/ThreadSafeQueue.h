#ifndef __THREADSAFEQUEUE_H__
#define __THREADSAFEQUEUE_H__

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
using namespace std::chrono_literals;

template < typename T >
class ThreadSafeQueue
{
  std::condition_variable cond;
  std::mutex mutex;
  std::queue < T > tsq;
  int maxSize;
public:

  ThreadSafeQueue () : maxSize ( 3000 ) {}

  ThreadSafeQueue ( int mxsz ) : maxSize ( mxsz ) {}

  void push ( T request )
  {
    std::unique_lock < std::mutex > lock ( mutex, std::defer_lock );
    lock.lock ();
    while ( isFull () )
    {
      std::this_thread::sleep_for ( 10ms );
    }
    tsq.push ( request );
    lock.unlock ();
  }

  void pop ( T &request )
  {
    std::unique_lock < std::mutex > lock ( mutex, std::defer_lock );
    lock.lock ();
    if ( ! isEmpty () )
    {
      request = tsq.front ();
      tsq.pop ();
    }
    lock.unlock ();
  }

  bool isFull () const
  {
    return tsq.size () >= maxSize;
  }

  bool isEmpty () const
  {
    return tsq.size () == 0;
  }

  int length () const
  {
    return tsq.size ();
  }

  void clear ()
  {
    std::unique_lock < std::mutex > lock ( mutex, std::defer_lock );
    lock.lock ();
    while ( !isEmpty () )
    {
      tsq.pop ();
    }
    lock.unlock ();
  }
};

#endif
