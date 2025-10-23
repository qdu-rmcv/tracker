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

        dst = queue[Back];
        Back = (Back+1)%queue.size();
        return true;
    }

    bool push(const T& src) 
    {
        if( (Front+1)%queue.size()  == Back ) { Back = (Back+1)%queue.size(); }
        
        queue[Front] = src;
        Front = (Front+1)%queue.size();
        return true;
    }

    bool empty() const { return Front==Back; }

    T& front() { return queue[ ( Front+queue.size()-1)%queue.size()]; }

    T& back() { return queue[Back]; }
    
    T& operator[](size_t index)  { return queue[(Back+index)%queue.size()]; }
    
    T& at(size_t index)  
    {   
        size_t i=(Back+index)%queue.size();
        if(!(Front>i&&i>=Back)||(Back>i&&i>=Front))
            throw std::out_of_range("Index out of range!");
        return queue[i];
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
