#include <iostream>
#include <vector>

int main()
{

    std::vector<int> ringBuffer;
    ringBuffer.reserve(3);
    for(int i=0; i<10; i++)
    {
        if(ringBuffer.size() >= 3)
            ringBuffer.erase(ringBuffer.begin());
        ringBuffer.emplace_back(i);
        // std::cout<<"Cap: " << ringBuffer.capacity() << " Size: " << ringBuffer.size() << std::endl;
        for (auto& val : ringBuffer)
        {
            std::cout << val << ", ";
        }
        std::cout << ";" << std::endl;
    }

    return 0;
}