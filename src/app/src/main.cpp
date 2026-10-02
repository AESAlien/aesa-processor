#include <iostream>
#include <thread>

// Using Thread Function
void print_hello(int id)
{
    std::cout << "Hello World from thread " << id << '\n';
}

int main()
{
    std::thread threads[5];

    int idx[5]; // Thread Id

    for (int i = 0; i < 5; ++i) {
        idx[i] = i + 1;
        // Create Thread
        threads[i] = std::thread(print_hello, idx[i]);
    }

    for (int i = 0; i < 5; ++i) {
        // Wait Thread
        threads[i].join();
    }

    return 0;
}