#ifndef RINGBUFFER_H
#define RINGBUFFER_H


template <typename T, int Capacity>
class RingBuffer 
{
public:
    RingBuffer() 
    {
        headIndex = 0;
        size_ = 0;
    }
    volatile T& operator[](int index) volatile;
    T read(int index) const volatile;
    void write(T value, int index) volatile;
    void push(T value) volatile;
    T pop() volatile;
    int size() const volatile
    {
        return size_;
    }
    int capacity() const volatile
    {
        return Capacity;
    }
	void clear() volatile
	{
		size_ = 0;
	}

private:
    T data[Capacity];
    int headIndex;
    int size_;
    inline int getRealIndex(int index) const volatile
    {
        return ((headIndex - index) % Capacity + Capacity) % Capacity;
    }
};


/* read(int index)
 *
 * Returns the object stored at a given index.
 */
template <typename T, int Capacity>
inline T RingBuffer<T, Capacity>::read(int index) const volatile
{
    return data[getRealIndex(index)];
}


/* operator[int index]
 *
 * Returns a reference to the object stored at a given index.
 */
template <typename T, int Capacity>
inline volatile T& RingBuffer<T, Capacity>::operator[](int index) volatile
{
    return data[getRealIndex(index)];
}


/* write(T value, int index)
 *
 * Sets the value of the object stored at the given index.
 */
template <typename T, int Capacity>
inline void RingBuffer<T, Capacity>::write(T value, int index) volatile
{
    data[getRealIndex(index)] = value;
}


/* push(T value)
 *
 * Pushes the given value to the head of the buffer.
 * Moves the head forward.
 * Overwrites the oldest value when size == capacity.
 */
template <typename T, int Capacity>
inline void RingBuffer<T, Capacity>::push(T value) volatile
{
    ++headIndex %= Capacity;
    data[headIndex] = value;
    if (size_ < Capacity)
        ++size_;
}


/* pop()
 *
 * Pops the top value off of the buffer and returns it.
 * Moves the head backward.
 * Calling pop() when size == 0 results in undefined behavior.
 */
template <typename T, int Capacity>
inline T RingBuffer<T, Capacity>::pop() volatile
{
    int oldHead = headIndex;
    headIndex = (headIndex - 1 + Capacity) % Capacity;
    if (size > 0)
        --size_;
    return data[oldHead]; 
}


#endif // RINGBUFFER_H