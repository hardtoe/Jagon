#ifndef VEC_H
#define VEC_H

#include <WProgram.h>
#include <math.h>

template<class Object, int size>
class Vec {
  private:
    Object values[size];
 
  public: 
    inline Object magnitude() {
      Object resultSquared = 0;
      
      for (int i = 0; i < size; i++) {
        if (!isnan(values[i])) {
          resultSquared +=
            sq(values[i]);
        }
      }
      
      return sqrt(resultSquared);
    }
  
    inline void operator=(Vec<Object,size> rhs) {
      for (int i = 0; i < size; i++) {
        values[i] = rhs.get(i);
      }
    }
  
    inline void operator=(Object newValues[]) {
      for (int i = 0; i < size; i++) {
        values[i] = newValues[i];
      }
    }
    
    inline Object maxReduce() {
      Object maxValue = values[0];
     
      for (int i = 1; i < size; i++) {
        maxValue = max(maxValue, values[i]);
      }
     
      return maxValue; 
    }
    
    inline byte maxReduceIndex() {
      Object maxValue = values[0];
      byte index = 0;
     
      for (int i = 1; i < size; i++) {
        if (values[i] > maxValue) {
          index = i;
          maxValue = values[i];
        }
      }
     
      return index; 
    }
    
    inline Vec<Object,size> absValue() {
      Vec<Object,size> value;
     
      for (int i = 0; i < size; i++) {
        value.values[i] = abs(values[i]);
      }
     
      return value; 
    }
    
    inline Vec<float,size> asFloat() {
      Vec<float,size> value;
     
      for (int i = 0; i < size; i++) {
        value.set(i, values[i]);
      }
     
      return value; 
    }
  
    inline Vec<Object,size> operator*(Object rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] * rhs); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator/(long rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] / rhs); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator/(float rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] / rhs); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator/(int rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] / rhs); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator+(Object rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] + rhs); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator-(Object rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] - rhs); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator*(Vec<long,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] * rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator*(Vec<int,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] * rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator*(Vec<float,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] * rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator/(Vec<int,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] / rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator/(Vec<float,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] / rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator/(Vec<long,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] / rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator+(Vec<Object,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] + rhs.get(i)); 
      }
      
      return result;
    }
    
    inline Vec<Object,size> operator-(Vec<Object,size> rhs) {
      Vec<Object,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, values[i] - rhs.get(i)); 
      }
      
      return result;
    }
    
    
    inline Vec<boolean,size> operator!=(Vec<Object,size> rhs) {
      Vec<boolean,size> result;
      
      for (int i = 0; i < size; i++) {
        result.set(i, ((int) values[i]) != ((int) rhs.get(i))); 
      }
      
      return result;
    }
    
    inline void set(int index, Object value) {
      values[index] = value;
    }   
    
    inline Object get(int index) {
      return values[index];
    }   
};

    inline Vec<long,4> operator/(long lhs, Vec<long,4> rhs) {
      Vec<long,4> result;
      
      for (int i = 0; i < 4; i++) {
        result.set(i, lhs / rhs.get(i)); 
      }
      
      return result;
    }

#endif // VEC_H
