#pragma once

#include <iostream>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

class console
{
private:
    /* data */
public:
    console(/* args */);
    ~console();

    void info(const char* msg);
    void warning(const char* msg);
    void error(const char* msg);

    bool enable;
    bool enable_info;
    bool enable_warning;
    bool enable_error;
};

console::console(/* args */)
{
    enable = true;
    enable_info = true;
    enable_warning = true;
    enable_error = true;
}

console::~console()
{
}


void console::info(const char* msg)
{
    if (enable_info && enable)
        std::cout << GREEN << msg << RESET << std::endl;
}

void console::warning(const char* msg)
{
    if (enable_warning && enable)
        std::cout << YELLOW << msg << RESET << std::endl;
}

void console::error(const char* msg)
{
    if (enable_error && enable)
        std::cout << RED << msg << RESET << std::endl;
}