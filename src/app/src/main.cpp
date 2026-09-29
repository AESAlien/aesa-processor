#include <comm/hello.hpp>
#include <pthread.h>
#include <iostream>

// Using Thread Function
void* print_hello(void* arg)
{
    const int id = *static_cast<int*>(arg);
    std::cout << "Hello World from thread " << id << '\n';
    return nullptr;
}

int main()
{
    comm::hello();

    pthread_t threads[5];

    int idx[5]; // Thread Id

    for (int i = 0; i < 5; ++i) {
        idx[i] = i + 1;
        // Create Thread
        pthread_create(&threads[i], nullptr, print_hello, &idx[i]);
    }

    for (int i = 0; i < 5; ++i) {
        // Wait Thread
        pthread_join(threads[i], nullptr);
    }

    return 0;
}