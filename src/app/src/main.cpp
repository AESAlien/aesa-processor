#include <iostream>
#include <thread>

// Using Thread Function
void PrintHello(int id)
{
    std::cout << "Hello World from thread " << id << '\n';
}

int main()
{
    std::thread threads[5];

    int idx[5]; // Thread Id

    for (int i = 0; i < 5; ++i)
    {
        idx[i] = i + 1;
        // Create Thread
        threads[i] = std::thread(PrintHello, idx[i]);
    }

    for (int i = 0; i < 5; ++i)
    {
        // Wait Thread
        threads[i].join();
    }

    return 0;
}