#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <WProgram.h>

#include "Assert.h"

template<class Object, int maxSize>
class CircularBuffer {
  private:
    volatile byte currentSize;
    byte enqueuePointer;
    byte dequeuePointer;
    Object buffer[maxSize];
  
  public:
    CircularBuffer() {
      this->currentSize = 0;
      this->enqueuePointer = 0;
      this->dequeuePointer = 0;
    }
    
    inline int size() {
      return currentSize;  
    }
    
    inline Object* create() {
      ASSERT(notFull());
      
      return &(buffer[enqueuePointer]);  
    }
    
    inline void put() {
      ASSERT(notFull());
      
      enqueuePointer =
        (enqueuePointer + 1) % maxSize;
          
      currentSize++;  
    }
    
    inline boolean notFull() {
      return currentSize < maxSize; 
    }
    
    inline boolean notEmpty() {
      return currentSize > 0;  
    }
    
    inline boolean isEmpty() {
      return currentSize == 0;  
    }
    
    inline Object* peek() {
      return &(buffer[dequeuePointer]);
    }
    
    inline Object* remove() {
      ASSERT(notEmpty());
      
      Object* removedObject = peek();
     
      currentSize--;
        
      dequeuePointer = 
        (dequeuePointer + 1) % maxSize;
      
      return removedObject;
    }
};


#endif // CIRCULAR_BUFFER_H
