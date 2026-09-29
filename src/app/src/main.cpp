#include <beam/hello.hpp>
#include <target/hello.hpp>

#include <iostream>

int main()
{
    beam::hello();
    target::hello();

    std::cout << "Press Enter to exit..." << std::flush;
    std::cin.get();
    return 0;
}
