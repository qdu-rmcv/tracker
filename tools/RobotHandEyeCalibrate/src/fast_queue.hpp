#ifndef INCLUDE_FAST_QUEUE_HPP
#define INCLUDE_FAST_QUEUE_HPP

#include <atomic>
#include <stdexcept>
#include <vector>


template <typename T>
  class FastQueue {
   public:


    explicit FastQueue(size_t max_queue_len)
        : queue(max_queue_len + 1),
          Front(0),
          Back(0){}

    bool pop(T& dst) 
    {
        if (Front == Back) return false;

        dst = std::move(queue[Back]);
        Back = (Back+1)%queue.size();
        return true;
    }

    bool push(const T& src) 
    {
        if( (Front+1)%queue.size()  == Back ) return false;
        
        queue[Front] = src;
        Front = (Front+1)%queue.size();
        return true;
    }

    bool push(T&& src) 
    {
        if( (Front+1)%queue.size()  == Back ) return false;
        
        queue[Front] = std::move(src);
        Front = (Front+1)%queue.size();
        return true;
    }

    bool empty() const { return Front==Back; }

    T& front() { return queue[ ( Front+queue.size()-1)%queue.size()]; }

    T& back() { return queue[Back]; }
    
    T& operator[](size_t index)  { return queue[(Back+index)%queue.size()]; }

    size_t size() const {return (Front-Back+queue.size())%queue.size();}
    
    T& at(size_t index)  
    {   
        size_t current_size=(Front-Back+queue.size())%queue.size();
        
        if(index >= current_size)
            throw std::out_of_range("Index out of range!");

        return queue[(Back + index) % queue.size()];
    }

    bool pop() 
    {
        if(Front == Back) return false;
        Back = (Back+1)%queue.size();
        return true;
    }


   private:
    std::vector<T> queue;
    std::atomic<size_t> Front,Back;
  };
#endif
