#include <iostream>
#include <pthread.h>
#include <unistd.h>

void* thread_func(void* arg)
{
    for (int i = 0; i < 5; i++)
    {
        std::cout << "Child thread running: " << i << std::endl;
        sleep(1);
    }

    return nullptr;
}

int main()
{
    pthread_t thread;

    int ret = pthread_create(&thread, nullptr, thread_func, nullptr);

    if (ret != 0)
    {
        std::cerr << "pthread_create failed" << std::endl;
        return 1;
    }

    for (int i = 0; i < 5; i++)
    {
        std::cout << "Main thread running: " << i << std::endl;
        sleep(1);
    }

    pthread_join(thread, nullptr);

    std::cout << "Thread finished" << std::endl;

    return 0;
}