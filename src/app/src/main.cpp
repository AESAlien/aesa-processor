#include <target/hello.hpp>
#include <comm/hello.hpp>

#include <iostream>

int main()
{
    target::hello();
    comm::hello();

    std::cout << "Press Enter to exit..." << std::flush;
    std::cin.get();
    return 0;
}
